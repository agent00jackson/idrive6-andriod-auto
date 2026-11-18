#ifndef NG_ALIVE_WIRE_H
#define NG_ALIVE_WIRE_H
#include <stdint.h>
#include <stddef.h>
#include <string.h>
/* New private local codec. Never a native model/Ready/receipt acknowledgement. */
#define NG_ALIVE_BYTES 48u
#define NG_ALIVE_ACK_BYTES 40u
#define NG_ALIVE_MAX_AGE_MS 5000u
#define NG_ALIVE_LEASE_MS 10000u
#define NG_ALIVE_ONLY 1u
#define NG_ALIVE_ENDPOINT_PREFIX "native-guidance-alive-"
#define NG_ALIVE_CLIENT_PREFIX "native-functional-alive-"
enum ng_alive_result { NG_ALIVE_ACCEPTED=0,NG_ALIVE_FRAME=1,NG_ALIVE_VALUE=2,NG_ALIVE_SESSION=3,NG_ALIVE_STALE=4,NG_ALIVE_CLOSED=5 };
struct ng_alive_message {uint64_t session;uint32_t semantic_sequence,heartbeat_sequence;uint64_t created_ms;};
struct ng_alive_ack {struct ng_alive_message message;uint32_t result,flags;uint64_t accepted_ms;};
static inline uint16_t ngk_r16(const unsigned char*p){return (uint16_t)(p[0]|((uint16_t)p[1]<<8));}
static inline uint32_t ngk_r32(const unsigned char*p){return (uint32_t)p[0]|((uint32_t)p[1]<<8)|((uint32_t)p[2]<<16)|((uint32_t)p[3]<<24);}
static inline uint64_t ngk_r64(const unsigned char*p){return ngk_r32(p)|((uint64_t)ngk_r32(p+4)<<32);}
static inline void ngk_w16(unsigned char*p,uint16_t x){p[0]=(unsigned char)x;p[1]=(unsigned char)(x>>8);}
static inline void ngk_w32(unsigned char*p,uint32_t x){for(unsigned i=0;i<4;++i)p[i]=(unsigned char)(x>>(8*i));}
static inline void ngk_w64(unsigned char*p,uint64_t x){ngk_w32(p,(uint32_t)x);ngk_w32(p+4,(uint32_t)(x>>32));}
static inline void ng_alive_encode(unsigned char p[NG_ALIVE_BYTES],const struct ng_alive_message*m){memset(p,0,NG_ALIVE_BYTES);memcpy(p,"NGK1",4);ngk_w16(p+4,1);ngk_w16(p+6,NG_ALIVE_BYTES);ngk_w64(p+8,m->session);ngk_w32(p+16,m->semantic_sequence);ngk_w32(p+20,m->heartbeat_sequence);ngk_w64(p+24,m->created_ms);}
static inline unsigned ng_alive_decode(const unsigned char*p,size_t n,struct ng_alive_message*m){memset(m,0,sizeof(*m));if(!p||n!=NG_ALIVE_BYTES||memcmp(p,"NGK1",4)||ngk_r16(p+4)!=1||ngk_r16(p+6)!=NG_ALIVE_BYTES||ngk_r64(p+32)||ngk_r64(p+40))return NG_ALIVE_FRAME;m->session=ngk_r64(p+8);m->semantic_sequence=ngk_r32(p+16);m->heartbeat_sequence=ngk_r32(p+20);m->created_ms=ngk_r64(p+24);return !m->session||!m->semantic_sequence||!m->heartbeat_sequence||!m->created_ms?NG_ALIVE_VALUE:NG_ALIVE_ACCEPTED;}
static inline void ng_alive_ack_encode(unsigned char p[NG_ALIVE_ACK_BYTES],const struct ng_alive_ack*a){memset(p,0,NG_ALIVE_ACK_BYTES);memcpy(p,"NKA1",4);ngk_w16(p+4,1);ngk_w16(p+6,NG_ALIVE_ACK_BYTES);ngk_w64(p+8,a->message.session);ngk_w32(p+16,a->message.semantic_sequence);ngk_w32(p+20,a->message.heartbeat_sequence);ngk_w32(p+24,a->result);ngk_w32(p+28,a->flags);ngk_w64(p+32,a->accepted_ms);}
static inline int ng_alive_ack_decode(const unsigned char*p,size_t n,struct ng_alive_ack*a){memset(a,0,sizeof(*a));if(!p||n!=NG_ALIVE_ACK_BYTES||memcmp(p,"NKA1",4)||ngk_r16(p+4)!=1||ngk_r16(p+6)!=NG_ALIVE_ACK_BYTES)return 0;a->message.session=ngk_r64(p+8);a->message.semantic_sequence=ngk_r32(p+16);a->message.heartbeat_sequence=ngk_r32(p+20);a->result=ngk_r32(p+24);a->flags=ngk_r32(p+28);a->accepted_ms=ngk_r64(p+32);return a->result<=NG_ALIVE_CLOSED&&(a->result==0?(a->flags==NG_ALIVE_ONLY&&a->accepted_ms):(a->flags==0&&!a->accepted_ms));}
static inline int ng_alive_ack_matches(const struct ng_alive_message*m,const struct ng_alive_ack*a){return a->result==NG_ALIVE_ACCEPTED&&a->flags==NG_ALIVE_ONLY&&a->message.session==m->session&&a->message.semantic_sequence==m->semantic_sequence&&a->message.heartbeat_sequence==m->heartbeat_sequence&&a->accepted_ms>=m->created_ms&&a->accepted_ms-m->created_ms<NG_ALIVE_MAX_AGE_MS;}
#endif
