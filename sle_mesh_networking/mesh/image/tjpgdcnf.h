/*----------------------------------------------*/
/* TJpgDec System Configurations R0.03          */
/* Adapted for SLE Mesh e-Paper project         */
/*----------------------------------------------*/

#define	JD_SZBUF		512	/* Size of stream input buffer */
#define JD_FORMAT		0	/* 0: RGB888 (3 byte/pix), 1: RGB565 (2 byte/pix), 2: Grayscale (1 byte/pix) */
#define	JD_USE_SCALE	0	/* 0: Disable, 1: Enable output scaling */
#define JD_TBLCLIP		1	/* 0: Disable, 1: Enable clipping table for saturation arithmetic */
#define JD_FASTDECODE	1	/* 0: Normal, 1: 32-bit barrel shifter, 2: Table-based huffman (fastest) */
