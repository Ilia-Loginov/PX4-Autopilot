//#include <drivers/drv_hrt.h>

#include <nuttx/mutex.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <lib/ringbuffer/Ringbuffer.hpp>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

namespace sensor_logger {
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
        explicit SensorLogger(); 
        void Start(const char * fileName);
        void Start(int tcp_port);
        void Stop();
        virtual ~SensorLogger();
        int WriteMessage(RegAccessPayload * message);
    
    private:
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
