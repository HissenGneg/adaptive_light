#include "boardcfg.h"
#include "lib/CAN/CAN.h"
#include "typedefs.h"
#include "common.h"

size_t can_id_to_index(uint8_t can_id)
{
    switch (can_id)
    {
    case CAN_ID_LED_1:
        return 0;
        break;
    case CAN_ID_LED_2:
        return 1;
        break;
    case CAN_ID_LED_3:
        return 2;
        break;
    case CAN_ID_LED_4:
        return 3;
        break;
    
    default:
        return -1;
        break;
    }
}

size_t index_to_can_id(uint8_t index)
{
    switch (index)
    {
    case 0:
        return CAN_ID_LED_1;
        break;
    case 1:
        return CAN_ID_LED_2;
        break;
    case 2:
        return CAN_ID_LED_3;
        break;
    case 3:
        return CAN_ID_LED_4;
        break;
    
    default:
        return -1;
        break;
    }
}