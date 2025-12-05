#include <unordered_map>
#include <iostream>
#include <time.h>
#include <pybind11/pybind11.h>

#include "piolib.h"
#include "ws2812.pio.h"

#define STRINGIFY(x) #x
#define MACRO_STRINGIFY(x) STRINGIFY(x)

namespace py = pybind11;

static PIO pio{};
static int offset{-1};
static struct timespec deadline;

constexpr auto NS_PER_SECOND = 1'000'000'000l;
constexpr auto NS_PER_MS = 1'000'000l;


//Data related to a state machine
struct SmData {
    int smId = -1; //state machine ID
    size_t lastSize = 0; // size of last data transferred to the state machine

    SmData() = default;
    SmData(int p_smid, size_t p_lastSize) : smId(p_smid), lastSize(p_lastSize) {}
};


//class to manage allocations of pio state machines to a gpio pin
class SmMgr {

    inline const static int MAX_SM_ALLOC = 4; // max amount state machine allocations allowed
    inline const static int SM_NA = -1; // returned by functions if state machine is not valid / not found

    private:
        std::unordered_map<int, SmData> smMap; // map of gpio::state machine allocations

    public:
        // Constructor
        SmMgr(){
            smMap.reserve(MAX_SM_ALLOC);
        }

        // Allocates a state machine to a gpio pin, if available
        // Returns associated state machine if already allocated
        int allocateSm(int gpio) {
            int tmpSm = getSm(gpio);
            if(tmpSm >= 0)
                return tmpSm;
            if (smMap.size() >= MAX_SM_ALLOC) {
                return SM_NA;
            }
            tmpSm = pio_claim_unused_sm(pio, true);
            smMap.try_emplace(gpio, tmpSm, 0);
            return tmpSm;
        }

        // Get state machine associated to a gpio pin, if available
        // Return -1 if not
        int getSm(int gpio) const {
            auto it = smMap.find(gpio);
            if(it != smMap.end()) {
                return it->second.smId;
            }
            return SM_NA;
        }

        // set size of last data transferred to sm associated with [gpio] if allocated
        // return 1 if successful, 0 if not found
        int setLastSize(int gpio, int value) {
            if(smMap.find(gpio) != smMap.end()) {
                smMap[gpio].lastSize = value;
                return 1;
            }
            else {
                return 0;
            }
        }

        // Get size of last data transferred to SM, if available
        // Return -1 if not available
        size_t getLastSize(int gpio) const {
            if(smMap.find(gpio) != smMap.end()) {
                return smMap.at(gpio).lastSize;
            }
            else {
                return SM_NA;
            }
        }

        // Free all state machines 
        void freeAllSm() {
            for(const auto& pair : smMap) {
                pio_sm_unclaim(pio, pair.second.smId);
            }
            smMap.clear();
        }
};

//instance state machine manager
static SmMgr smManager;


static void timespec_add_ns(struct timespec &out, const struct timespec &in, long ns) {
    out = in;
    out.tv_nsec += ns;
    if(out.tv_nsec > NS_PER_SECOND) {
        out.tv_nsec -= NS_PER_SECOND;
        out.tv_sec += 1;
    }
}


static void neopixel_write(py::object gpio_obj, py::buffer buf) {
    int gpio = py::getattr(gpio_obj, "_pin", gpio_obj).attr("id").cast<int>();
    py::buffer_info info = buf.request();
    int sm = smManager.getSm(gpio);
    if (!pio || sm < 0) {
        if(!pio) {
            // (is safe to call twice)
            if (pio_init()) {
                throw std::runtime_error("pio_init() failed");
            }
    
            // can't use `pio0` macro as it will call exit() on failure!
            pio = pio_open(0);
            if (PIO_IS_ERR(pio)) {
                throw std::runtime_error(
                    py::str("Failed to open PIO device (error {})")
                        .attr("format")(PIO_ERR_VAL(pio))
                        .cast<std::string>());
            }

            offset = pio_add_program(pio, &ws2812_program);
        }
        if(sm<0) {
            sm = smManager.allocateSm(gpio);
            pio_sm_clear_fifos(pio, sm);
            ws2812_program_init(pio, sm, offset, gpio, 800000.0, true);
        }
    } else {
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &deadline, NULL);
    }

    size_t size = info.size * info.itemsize;

    // rp1pio can only DMA in 32-bit blocks.
    // Copy the data into a temporary vector, with redundant zeros at the end
    // then byteswap it so that the data comes out in the right order.
    uint8_t *data = reinterpret_cast<uint8_t *>(info.ptr);
    std::vector<uint32_t> vec;
    vec.resize((size + 3) / 4);
    size_t data_size = vec.size() * 4;
    memcpy(&vec[0], data, size);
    for (auto &i : vec)
        i = __builtin_bswap32(i);

    if (data_size > UINT16_MAX) {
        throw py::value_error("Too much data");
    }
    if (data_size != smManager.getLastSize(gpio)) {
        if (pio_sm_config_xfer(pio, sm, PIO_DIR_TO_SM, data_size, 1)) {
            throw std::runtime_error("pio_sm_config_xfer() failed");
        }
        smManager.setLastSize(gpio, data_size);
    }
    if (pio_sm_xfer_data(pio, sm, PIO_DIR_TO_SM, data_size, &vec[0])) {
        throw std::runtime_error("pio_sm_xfer_data() failed");
    }

    // Track the earliest time at which we can start a fresh neopixel transmission.
    // This needs to be long enough that all bits have been clocked out (the FIFO can
    // contain 16 entries of 32 bits each, taking 640us to transmit) plus the ws2812
    // required idle time ("RET code") of 50usmin. These sum to around 700us, so the 1ms
    // delay is generous.
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    timespec_add_ns(deadline, ts, NS_PER_MS);
}

static void free_pio(void) {
    if (!pio) {
        return;
    }
    if (offset <= 0) {
        pio_remove_program(pio, &ws2812_program, offset);
    };
    offset = -1;

    smManager.freeAllSm();

    pio_close(pio);
    pio = nullptr;
}

PYBIND11_MODULE(adafruit_raspberry_pi5_neopixel_write, m) {
    m.doc() = R"pbdoc(
        neopixel_write for pi5
        ----------------------

        .. currentmodule:: adafruit_raspberry_pi5_neopixel_write

        .. autosummary::
           :toctree: _generate

           neopixel_write
           free_pio
    )pbdoc";

    m.def("neopixel_write", &neopixel_write, py::arg("gpio"), py::arg("buf"),
          R"pbdoc(NeoPixel writing function)pbdoc");

    m.def("free_pio", &free_pio, R"pbdoc(Release any held PIO resource)pbdoc");

#ifdef VERSION_INFO
    m.attr("__version__") = MACRO_STRINGIFY(VERSION_INFO);
#else
    m.attr("__version__") = "dev";
#endif
}