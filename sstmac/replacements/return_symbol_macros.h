
#ifdef sstmac_must_return_free
#define free sstmac_free
#undef sstmac_must_return_free
#endif

#ifdef sstmac_must_return_memset
#undef memset
#define memset sstmac_memset
#undef sstmac_must_return_memset
#endif

