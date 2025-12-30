
#include <px4_platform_common/sensor_logger/SensorLogger.hpp>

#include <fcntl.h>      // open(), O_CREAT, O_WRONLY, O_TRUNC
#include <unistd.h>    // close(), write()
#include <sys/stat.h>  
#include <px4_platform_common/px4_config.h>  // PX4_O_MODE_666
#include <px4_platform_common/log.h>
#include <drivers/drv_hrt.h>



#include <sys/socket.h>
#include <assert.h>
#include <netinet/in.h>
#include <arpa/inet.h>


namespace sensor_logger {
	using namespace time_literals;

	SensorLogger::SensorLogger():
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
	{
		ScheduleOnInterval(10_ms);
		pthread_mutex_init(&_mutex, NULL);
		_ring_buffer.allocate(sizeof(RegAccessPayload)*50);
	}

	void SensorLogger::Start(const char * fileName) {
		if (_started)
		{
			PX4_WARN("Attempt for start file logging. SensorLogger already started");
			return;
		}

		_log_fd = ::open(fileName, O_CREAT |  O_WRONLY | O_TRUNC, PX4_O_MODE_666);
		if (_log_fd < 0) {
			PX4_ERR("Can't open log file %s", fileName);
		}
		else {
			_started = true;
			_backend = BackendType::FILE;
			PX4_INFO("Logger was started in FILE mode");
		}
	}

	void SensorLogger::Start(int tcp_port) {
		if (_started) {
			PX4_ERR("Attempt for start file logging. SensorLogger already started");
			return;
		}
		_tcp_socket = socket(AF_INET, SOCK_STREAM, 0);

		int opt = 1;
		setsockopt(_tcp_socket, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

		sockaddr_in addr{};
		addr.sin_family = AF_INET;
		addr.sin_port   = htons(tcp_port);
		addr.sin_addr.s_addr = htonl(INADDR_ANY);

		bind(_tcp_socket, (struct sockaddr *)&addr, sizeof(addr));
		listen(_tcp_socket, 1);

		// non-blocking
		fcntl(_tcp_socket, F_SETFL, O_NONBLOCK);
		_started = true;		
		_backend = BackendType::TCP;
		PX4_INFO("Logger was started in TCP mode");
	}

	void SensorLogger::Stop() {
		if (_started) {
			if (_backend == BackendType::TCP) {
				CloseDescriptorIfOpen(_client_socket);
				CloseDescriptorIfOpen(_tcp_socket);
			}
			else if (_backend == BackendType::FILE) {
				CloseDescriptorIfOpen(_log_fd);
			}
			_started = -1;
			PX4_INFO("Logger was stopped");
		}
	}

	void SensorLogger::CloseDescriptorIfOpen(int& fd) {
		if (fd > 0) {
			close(fd);
			fd = -1;
		}
	}
		
	int SensorLogger::WriteMessage(RegAccessPayload * message) {
		pthread_mutex_lock(&_mutex);
		message->timestamp = hrt_absolute_time();
		bool res = _ring_buffer.push_back(reinterpret_cast<uint8_t *>(message), sizeof(RegAccessPayload));
		if (!res)
		{
			PX4_ERR("buffer is full. time %llu" , message->timestamp);
		}
		pthread_mutex_unlock(&_mutex);
		return res ? sizeof(RegAccessPayload) : 0;
	}

	void SensorLogger::Run(){

		if (! _started)
			return;

		if (_backend == BackendType::TCP && _client_socket < 0) {
			_client_socket = accept(_tcp_socket, nullptr, nullptr);
			if (_client_socket >= 0) {
				fcntl(_client_socket, F_SETFL, O_NONBLOCK);
				PX4_INFO("TCP client connected");
			}
		}

		while (true) {
			RegAccessPayload pkt;
			pthread_mutex_lock(&_mutex);
			
			size_t size = _ring_buffer.pop_front(reinterpret_cast<uint8_t *>(&pkt), sizeof(pkt));
			pthread_mutex_unlock(&_mutex);
			if (0 == size) {
				break;
			}			

			char message[100] {};
			int message_len = sprintf(message, "log module %s tms=%llu, reg=%x, value=%x, op=%s\n",
				pkt.unit_name, pkt.timestamp, pkt.reg, pkt.value, pkt.op == RegOp::READ ? "read" : "write");
			if ( message_len== -1) {
				PX4_ERR("Fail of generation message for logging");
				continue;
			}
			
			if (_backend == BackendType::TCP) {
				if (_client_socket >= 0) {
					ssize_t ret = send(_client_socket,
									message,
									message_len,
									MSG_DONTWAIT);
					if (ret < 0) {
						PX4_WARN("TCP client disconnected");
						close(_client_socket);
						_client_socket = -1;
					}
				}
			}
			else if (_backend == BackendType::FILE) {
				ssize_t ret = ::write(_log_fd, message, message_len);
				if (ret < 0) {
					PX4_WARN("File is not available for writing");
					::close(_log_fd);
					_log_fd = -1;
					_started = false;
				}

			}			
		}
	}

	SensorLogger::~SensorLogger() {
		pthread_mutex_destroy(&_mutex);
		ScheduleClear();
		Stop();
	}

}

