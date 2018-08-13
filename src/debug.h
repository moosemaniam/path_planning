#ifndef DEBUG_H
#define DEBUG_H
//#define DEBUG_PRINT


#ifdef DEBUG_PRINT
#define LOGD(fmt, args...)  \
       fprintf(stderr, "%s:%d:%s():\n" fmt, __FILE__, __LINE__, __FUNCTION__, ##args)
#else
#define LOGD(fmt, args...)    /* Don't do anything in non-debug builds */
#endif
#endif
