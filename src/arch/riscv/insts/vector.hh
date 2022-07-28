#include <stdint.h>

namespace RiscvISA
{
    uint8_t *getVectorRegister(uint8_t registerIndex);

    uint8_t getElemSize();

    void setElemSize(uint8_t elem);

    void setElemCount(uint8_t elem);

    uint8_t getElemCount();

    const int VecSize = 256;
    const int VecSizeElem = VecSize / 8;
} // nam