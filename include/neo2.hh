#pragma once
#include <ee/ee.hh>
#include <bus/bus.hh>
#include <log/log.hh>
#include <cpu/disassembler.hh>
#include <iop/iop.hh>

class Neo2 {
  public:
    enum class Subsystem {
        None,
        EE,
        IOP,
        Bus,
        Disassembler,
        Frontend
    };

    Neo2(std::shared_ptr<LogBackend> logger = nullptr);
    virtual ~Neo2();

    virtual void init() = 0;
    virtual void run(int argc, char **argv) = 0;

    static int exit(int code, Subsystem subsystem);
    static bool is_aborted() { return aborted; }
    static void reset_aborted() {
        aborted = false;
        guilty_subsystem = Subsystem::None;
    }

  static Subsystem get_guilty_subsystem() { return guilty_subsystem; }

  static void pause_emulation() {
        std::lock_guard<std::mutex> lock(emulation_mutex);
        emulation_paused = true;
    }

    static void resume_emulation() {
        std::lock_guard<std::mutex> lock(emulation_mutex);
        emulation_paused = false;
    }

    static bool is_emulation_paused() {
        std::lock_guard<std::mutex> lock(emulation_mutex);
        return emulation_paused;
    }

    EE ee;
    IOP iop;
    Bus bus;
    Disassembler disassembler;

  private:
    static inline bool aborted = false;
    static inline Subsystem guilty_subsystem = Subsystem::None;
    static inline std::mutex emulation_mutex;
    static inline bool emulation_paused = false;
};

#define RESET "\033[0m"
#define BLACK "\033[30m"              /* Black */
#define RED "\033[31m"                /* Red */
#define GREEN "\033[32m"              /* Green */
#define YELLOW "\033[33m"             /* Yellow */
#define BLUE "\033[34m"               /* Blue */
#define MAGENTA "\033[35m"            /* Magenta */
#define CYAN "\033[36m"               /* Cyan */
#define WHITE "\033[37m"              /* White */
#define BOLDBLACK "\033[1m\033[30m"   /* Bold Black */
#define BOLDRED "\033[1m\033[31m"     /* Bold Red */
#define BOLDGREEN "\033[1m\033[32m"   /* Bold Green */
#define BOLDYELLOW "\033[1m\033[33m"  /* Bold Yellow */
#define BOLDBLUE "\033[1m\033[34m"    /* Bold Blue */
#define BOLDMAGENTA "\033[1m\033[35m" /* Bold Magenta */
#define BOLDCYAN "\033[1m\033[36m"    /* Bold Cyan */
#define BOLDWHITE "\033[1m\033[37m"   /* Bold White */
