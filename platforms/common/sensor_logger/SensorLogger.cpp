
#include <px4_platform_common/sensor_logger/SensorLogger.hpp>

#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>
#include <px4_platform_common/px4_config.h>  // PX4_O_MODE_666
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/log.h>
#include <drivers/drv_hrt.h>

#include <sys/socket.h>
#include <assert.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <errno.h>
namespace sensor_logger {
	using namespace time_literals;

	SensorLogger::SensorLogger():
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
	{
		pthread_mutex_init(&_mutex, NULL);
		param_t pf = param_find("SL_MODE");
		int32_t value = 0;
		if (pf != PARAM_INVALID && param_get(pf, &value) == PX4_OK) {
			if (value > static_cast<int32_t>(BackendType::FILE) ||
			    static_cast<int32_t> (BackendType::OFF) > value) {
					PX4_ERR("SL_MODE is out of range!");
				}
				else {
					_backend = static_cast<BackendType> (value);
				}
		}
		
		if (BackendType::OFF == _backend)
			return;
		
		pf = param_find("SL_MAX_MSG");
		size_t max_msg = 50; 
		if (pf != PARAM_INVALID && param_get(pf, &value) == PX4_OK) {
			max_msg = value; 
		}
		_ring_buffer.allocate(sizeof(RegAccessPayload)*max_msg);

		if (BackendType::FILE == _backend) {
			Start(DEFAULT_LOG_FILE);
		}
		else if (BackendType::TCP == _backend) {
			Start(DEFAULT_TCP_PORT);
		}
		
		ScheduleOnInterval(10_ms);
	}

	void SensorLogger::Start(const char * fileName) {
		pthread_mutex_lock(&_mutex);
		if (isAllowedForStart()) {
			_log_fd = ::open(fileName, O_CREAT | O_APPEND | O_WRONLY | O_TRUNC, PX4_O_MODE_666);
			if (_log_fd < 0) {
				PX4_ERR("Can't open log file %s", fileName);
			}
			else {
				_started.store(true);
				_backend = BackendType::FILE;
				PX4_INFO("Logger was started in FILE mode");
			}
		}
		pthread_mutex_unlock(&_mutex);
	}

	void SensorLogger::Start(int tcp_port) {
		pthread_mutex_lock(&_mutex);
		if (isAllowedForStart()) {
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
			_started.store(true);		
			_backend = BackendType::TCP;
			PX4_INFO("Logger was started in TCP mode");
		}
		pthread_mutex_unlock(&_mutex);
	}

	void SensorLogger::Stop() {
		pthread_mutex_lock(&_mutex);
		if (_started.load()) {
			if (_backend == BackendType::TCP) {
				CloseDescriptorIfOpen(_client_socket);
				CloseDescriptorIfOpen(_tcp_socket);
			}
			else if (_backend == BackendType::FILE) {
				CloseDescriptorIfOpen(_log_fd);
			}
			_started.store(false);
			PX4_INFO("Logger was stopped");
		}
		pthread_mutex_unlock(&_mutex);
	}

	void SensorLogger::CloseDescriptorIfOpen(int& fd) {
		if (fd >= 0) {
			close(fd);
			fd = -1;
		}
	}

	bool SensorLogger::isAllowedForStart() const {
		if (BackendType::OFF == _backend) {
			PX4_WARN("Sensor logger is turned off");
			return false;
		}

		if (_started.load())
		{
			PX4_WARN("Attempt for start file logging. SensorLogger already started");
			return false;
		}
		return true;
	}
		
	int SensorLogger::WriteMessage(const char* unit, uint8_t reg, uint8_t value, RegOp op) {
		
		if (!unit || !_started.load()) 
			return 0;

		pthread_mutex_lock(&_mutex);
		RegAccessPayload message{"", hrt_absolute_time(), reg, value, op};
		size_t len = strlen(unit);
		if (len >= sizeof(message.unit_name)) {
			memcpy(message.unit_name, unit, sizeof(message.unit_name) - 1);
			message.unit_name[sizeof(message.unit_name)-1] = '\0';
		}
		else {
			memcpy(message.unit_name, unit,len);
		}

		bool res = _ring_buffer.push_back(reinterpret_cast<uint8_t *>(&message), sizeof(RegAccessPayload));
		if (!res) {
			PX4_ERR("buffer is full. time %llu" , message.timestamp);
		}
		
		pthread_mutex_unlock(&_mutex);
		return res ? sizeof(RegAccessPayload) : 0;
	}

	void SensorLogger::Run(){

		if (! _started.load())
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
			int message_len = snprintf(message, sizeof(message),"log module %s tms=%llu, reg=%x, value=%x, op=%s\n",
				pkt.unit_name, pkt.timestamp, pkt.reg, pkt.value, pkt.op == RegOp::READ ? "read" : "write");
			if ( message_len < 0 || message_len >= (int)sizeof(message)) {
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
						if (errno == EWOULDBLOCK || errno == EAGAIN) {
							// skip
						}
						else {
							PX4_WARN("TCP client disconnected");
							close(_client_socket);
							_client_socket = -1;
						}
					}
				}
			}
			else if (_backend == BackendType::FILE) {
				ssize_t ret = ::write(_log_fd, message, message_len);
				if (ret < 0) {
					PX4_WARN("File is not available for writing");
					::close(_log_fd);
					_log_fd = -1;
					_started.store(false);
				}

			}			
		}
	}

	SensorLogger::~SensorLogger() {
		Stop();
		ScheduleClear();
		pthread_mutex_destroy(&_mutex);
	}

}

