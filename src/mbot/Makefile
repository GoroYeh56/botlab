include ../common.mk

CFLAGS  =  $(CFLAGS_STD)  $(CFLAGS_COMMON)  $(CFLAGS_LCMTYPES) -O3
CXXFLAGS = $(CXXFLAGS_STD) $(CFLAGS_COMMON)  $(CFLAGS_LCMTYPES) -O3
LDFLAGS = $(LDFLAGS_STD) $(LDFLAGS_COMMON) $(LDFLAGS_LCMTYPES) $(LDFLAGS_RPLIDAR) 
LIBDEPS = $(call libdeps, $(LDFLAGS))

BIN_DRIVE_SQUARE = $(BIN_PATH)/drive_square
BIN_RPLIDAR_DRIVER = $(BIN_PATH)/rplidar_driver
BIN_MOTION_CONTROLLER = $(BIN_PATH)/motion_controller
BIN_TIMESYNC = $(BIN_PATH)/timesync
BIN_WIFI_READER = $(BIN_PATH)/wifi_reader
BIN_OLED_EXAMPLE = $(BIN_PATH)/oled_example
BIN_CHECKPOINT1 = $(BIN_PATH)/checkpoint1
BIN_CHECKPOINT2 = $(BIN_PATH)/checkpoint2
BIN_COMP2 = $(BIN_PATH)/competition_event2
BIN_MOTION_CONTROLLER_EVENT1 = $(BIN_PATH)/motion_controller_event1
BIN_EVENT2 = $(BIN_PATH)/event_2



ALL = $(BIN_RPLIDAR_DRIVER) $(BIN_SWEEP_DRIVER) $(BIN_WIFI_READER) $(BIN_TIMESYNC) $(BIN_DRIVE_SQUARE) $(BIN_MOTION_CONTROLLER) $(BIN_MC_SMOOTH) $(BIN_OLED_EXAMPLE) $(BIN_CHECKPOINT1) $(BIN_CHECKPOINT2) $(BIN_COMP2) $(BIN_MOTION_CONTROLLER_EVENT1) $(BIN_EVENT2)

all: $(ALL)

laptop-only: $(BIN_TIMESYNC) $(BIN_DRIVE_SQUARE) $(BIN_MOTION_CONTROLLER) $(BIN_CHECKPOINT1) $(BIN_CHECKPOINT2) $(BIN_COMP2) $(BIN_MOTION_CONTROLLER_EVENT1) $(BIN_EVENT2)

$(BIN_DRIVE_SQUARE): drive_square.o $(LIBDEPS)
	@echo "    $@"
	@$(CXX) -o $@ $^ $(LDFLAGS)

$(BIN_MOTION_CONTROLLER): motion_controller.o $(LIBDEPS)
	@echo "    $@"
	@$(CXX) -o $@ $^ $(LDFLAGS)

$(BIN_RPLIDAR_DRIVER): rplidar_driver.o $(LIBDEPS)
	@echo "    $@"
	@$(CXX) -o $@ $^ $(LDFLAGS)

$(BIN_TIMESYNC): timesync.o $(LIBDEPS)
	@echo "    $@"
	@$(CXX) -o $@ $^ $(LDFLAGS)

$(BIN_WIFI_READER): wifi_reader.o $(LIBDEPS)
	@echo "    $@"
	@$(CC) -o $@ $^ $(LDFLAGS)

$(BIN_OLED_EXAMPLE): oled_example.o $(LIBDEPS)
	@echo "    $@"
	@$(CXX) -o $@ $^ $(LDFLAGS)

$(BIN_CHECKPOINT1): checkpoint1.o $(LIBDEPS)
	@echo "    $@"
	@$(CXX) -o $@ $^ $(LDFLAGS)
$(BIN_CHECKPOINT2): checkpoint2.o $(LIBDEPS)
	@echo "    $@"
	@$(CXX) -o $@ $^ $(LDFLAGS)

$(BIN_COMP2): competition_event2.o $(LIBDEPS)
	@echo "    $@"
	@$(CXX) -o $@ $^ $(LDFLAGS)
$(BIN_MOTION_CONTROLLER_EVENT1): motion_controller_event1.o $(LIBDEPS)
	@echo "    $@"
	@$(CXX) -o $@ $^ $(LDFLAGS)

$(BIN_EVENT2): event_2.o $(LIBDEPS)
	@echo "    $@"
	@$(CXX) -o $@ $^ $(LDFLAGS)

clean:
	@rm -f *.o *~ *.a
	@rm -f $(ALL)
