static void unpack_iq(unsigned n) {
    const uint8_t *base=(uint8_t *)IQ_BUFFER;
    for(unsigned pair=(n+1)/2;pair-->0;) {
        const uint8_t *p=base+pair*5;
        uint32_t a=p[0]|((uint32_t)p[1]<<8)|((uint32_t)(p[2]&15)<<16);
        if(pair*2+1<n) {
            uint32_t b=(p[2]>>4)|((uint32_t)p[3]<<4)|((uint32_t)p[4]<<12);
            IQ_BUFFER[pair*2+1]=b;
        }
        IQ_BUFFER[pair*2]=a;
    }
}
static void unpack_iq8(unsigned n) {
    const uint8_t *p=(const uint8_t *)IQ_BUFFER;
    for(unsigned j=n;j-->0;) {
        uint32_t i=p[2*j],q=p[2*j+1];IQ_BUFFER[j]=(i<<2)|(q<<12);
    }
}
