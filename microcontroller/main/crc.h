
#ifndef CM_DONE
#define CM_DONE

#ifndef FALSE
#define FALSE 0
#endif

#ifndef TRUE
#define TRUE 1
#endif

/**
 * Example usage for CRC-16/CCITT-False with poly=16
 * cm_t cm;
 * const unsigned char *msg = (const unsigned char *) "123456789";
 * unsigned long crc;
 * 
 * cm.cm_width = 16;
 * cm.cm_poly = 0x1021L;
 * cm.cm_init = 0xFFFFL;
 * cm.cm_refin = FALSE;
 * cm.cm_refot = FALSE;
 * cm.cm_xorot = 0x0000L;
 * 
 * cm_ini(&cm); 
 * cm_blk(&cm, msg, 9); 
 * crc = cm_crc(&cm);
 * 
 */

typedef struct
{
    int cm_width;
    unsigned long cm_poly; 
    unsigned long cm_init; 
    unsigned cm_refin;
    unsigned cm_refot; 
    unsigned long cm_xorot; 
    unsigned long cm_reg;
} cm_t;

typedef cm_t *p_cm_t;

#ifdef __cplusplus
extern "C" {
#endif

void cm_ini(p_cm_t p_cm);
void cm_nxt(p_cm_t p_cm, int ch);
void cm_blk(p_cm_t p_cm, const unsigned char *blk_adr, unsigned long blk_len);
unsigned long cm_crc(p_cm_t p_cm);
unsigned long cm_tab(p_cm_t p_cm, int index);

#ifdef __cplusplus
}
#endif

#endif
