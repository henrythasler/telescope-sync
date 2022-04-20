#ifndef NEXSTAR_H
#define NEXSTAR_H

#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h> 

#include <telescope.h>
#include <gnss.h>

class NexStar
{
public:
    NexStar(Telescope *telescope, GNSS *gnss);
    uint32_t handleRequest(const uint8_t *request, size_t requestLength, uint8_t *response, size_t responseMaxLength);

private:    
    Telescope *telescope;
    GNSS *gnss;
};
#endif // NEXSTAR_H