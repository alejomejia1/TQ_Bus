

#define SBUS_FROM_MASTER_PACKET_HEADER			0xAC    // communication packet header
#define SBUS_FROM_MASTER_PACKET_FOOTER			0xAD    // communication packet footer
#define SBUS_FROM_MASTER_PACKET_ESCAPE			0xAE    // escape character for handling occurrences of header, footer and this escape bytes in original message
#define SBUS_FROM_MASTER_PACKET_ESCAPE_MASK	0x80    // byte after ESCAPE character should be XOR'd with this value
#define SBUS_FROM_MASTER_FAILSAFE_MASK			0X08

void parse_sbus_from_master_packet();
void decode_sbus_from_master_info();
