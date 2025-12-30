
px4_add_library(sensor_logger
	SensorLogger.cpp
)

target_compile_options(sensor_logger PRIVATE ${MAX_CUSTOM_OPT_LEVEL})

target_link_libraries(sensor_logger
	PRIVATE
		px4_work_queue
		ringbuffer
)