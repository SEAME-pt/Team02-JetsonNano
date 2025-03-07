#include <vector>
#include <cstring>
#include <stdexcept>
#include <linux/joystick.h>
#include <cstdint>

static std::vector<uint8_t> mockData;
static std::vector<uint8_t> writtenData;
static int mockFd = 1;

extern "C" int custom_xbox_open(const char* path, int flags) {
    return mockFd;
}

extern "C" int custom_xbox_close(int fd) {
    return 0;
}

extern "C" int custom_xbox_ioctl(int fd, unsigned long request, int* arg) {
    if (request == JSIOCGAXES) {
        *arg = 2; // Mock 2 axes
        return 0;
    } else if (request == JSIOCGBUTTONS) {
        *arg = 10; // Mock 10 buttons
        return 0;
    }
    return -1;
}

extern "C" ssize_t custom_xbox_read(int fd, void* buf, size_t count) {
    if (mockData.size() < count) {
        throw std::runtime_error("Not enough mock data available");
    }
    std::memcpy(buf, mockData.data(), count);
    mockData.erase(mockData.begin(), mockData.begin() + count);
    return count;
}

extern "C" ssize_t custom_xbox_write(int fd, const void* buf, size_t count) {
    writtenData.insert(writtenData.end(), (uint8_t*)buf, (uint8_t*)buf + count);
    return count;
}

void custom_xbox_setMockData(const std::vector<uint8_t>& data) {
    mockData = data;
}

std::vector<uint8_t> custom_xbox_getWrittenData() {
    return writtenData;
}