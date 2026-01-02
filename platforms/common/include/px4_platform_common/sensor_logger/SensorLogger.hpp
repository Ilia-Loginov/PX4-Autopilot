//#include <drivers/drv_hrt.h>

#include <nuttx/mutex.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <lib/ringbuffer/Ringbuffer.hpp>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

namespace sensor_logger {

    constexpr static int DEFAULT_TCP_PORT = 32587;
    constexpr static char DEFAULT_LOG_FILE[] = PX4_STORAGEDIR "/sensor_log.txt";
    enum class RegOp : uint8_t {
        READ  = 0,
        WRITE = 1,
    };

    struct RegAccessPayload {
        char unit_name[10];
        uint64_t timestamp;
        uint8_t reg;
        uint8_t value;
        RegOp   op;
    };

    enum class BackendType {
        NONE,
        FILE,
        TCP
    };

    
    class SensorLogger : public px4::ScheduledWorkItem {
    public:
        //static singleton
        [[nodiscard]] static SensorLogger& get_instance() {
            static SensorLogger instance;
            return instance;
        }
        SensorLogger(const SensorLogger&) = delete;
        SensorLogger& operator=(const SensorLogger&) = delete;

        void Start(const char * fileName);
        void Start(int tcp_port);
        void Stop();
        int WriteMessage(RegAccessPayload * message);
        int WriteMessage(const char * unit, uint8_t reg, uint8_t value, RegOp op);
    
    private:
        SensorLogger(); 
        virtual ~SensorLogger();
        void Run() override;
        void CloseDescriptorIfOpen(int& fd);
        bool _started{};
        int _log_fd{};  
        int _tcp_socket{-1};    
        int _client_socket{-1};

        BackendType _backend;
        Ringbuffer _ring_buffer {};
        pthread_mutex_t _mutex;
    };

}
