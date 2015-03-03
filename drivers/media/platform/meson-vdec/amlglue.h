#include "../../../amlogic/amports/streambuf.h"
#include "../../../amlogic/amports/esparser.h"
#include "../../../amlogic/amports/amports_priv.h"

#define PARSER_VIDEO        (ES_TYPE_VIDEO)

stream_port_t *amstream_find_port(const char *name);
void amstream_port_open(stream_port_t *this);
int video_port_init(stream_port_t *port, struct stream_buf_s * pbuf);

void esparser_start_search(u32 parser_type, u32 phys_addr, u32 len);
