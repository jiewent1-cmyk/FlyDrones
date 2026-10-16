# libvl53l7cx.so: ST ULD (../st, unchanged, BSD-3) + Linux i2c-dev platform (this directory)
CXXFLAGS ?= -O2 -fPIC -Wall -Wno-unused-variable
libvl53l7cx.so: vl53l7cx_linux.cpp ../st/vl53l7cx_api.cpp
	$(CXX) $(CXXFLAGS) -I. -I../st -shared -o $@ vl53l7cx_linux.cpp ../st/vl53l7cx_api.cpp
clean:
	rm -f libvl53l7cx.so
