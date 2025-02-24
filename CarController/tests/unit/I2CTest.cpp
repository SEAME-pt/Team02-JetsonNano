#include <gtest/gtest.h>
#include "I2C.hpp"

class I2CTest : public ::testing::Test {
protected:
    void SetUp() override {
        i2c = new I2C();
    }

    void TearDown() override {
        delete i2c;
    }

    I2C* i2c;
};

TEST_F(I2CTest, InitializationTest) {
    EXPECT_NO_THROW(i2c->init("/dev/i2c-1"));
}

TEST_F(I2CTest, WriteByteTest) {
    i2c->init("/dev/i2c-1");
    EXPECT_NO_THROW(i2c->writeByte(0x40, 0x00, 0x10));
}

TEST_F(I2CTest, ReadByteTest) {
    i2c->init("/dev/i2c-1");
    EXPECT_NO_THROW(i2c->readByte(0x40));
}