#include "vector.hh"

uint8_t vect[][RiscvISA::VecSizeElem] = {
    {}, // v0
    {}, // v1
    {}, // v2
    {}, // v3
    {}, // v4
    {}, // v5
    {}, // v6
    {}, // v7
    {}, // v8
    {}, // v9
    {}, // v10
    {}, // v11
    {}, // v12
    {}, // v13
    {}, // v14
    {}, // v15
    {}, // v16
    {}, // v17
    {}, // v18
    {}, // v19
    {}, // v20
    {}, // v21
    {}, // v22
    {}, // v23
    {}, // v24
    {}, // v25
    {}, // v26
    {}, // v27
    {}, // v28
    {}, // v29
    {}, // v30
    {}, // v31
};

uint8_t elemSize = 1;

uint8_t elemCount = 0;

namespace RiscvISA
{
    uint8_t *getVectorRegister(uint8_t registerIndex)
    {
        return vect[registerIndex];
    }

    uint8_t getElemSize(){
        return elemSize;
    }

    void setElemSize(uint8_t elem){
        elemSize=elem;
    }

    void setElemCount(uint8_t elem)
    {
        elemCount = elem;
    }

    uint8_t getElemCount()
    {
        return elemCount;
    }
}