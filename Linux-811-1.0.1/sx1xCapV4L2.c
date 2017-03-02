/* This sample program is based on the "capture.c", a V4L2 video capture example,
 * in v4l2spec package that is provided by http://v4l2spec.bytesex.org/. The
 * image-saving-to-file portion is ported from sx11ppm.c and sx11pgm.c written 
 * by Charlie X. Liu at Sensoray Co.
 *
 * The sample program is only designed and could be used for demonstrating
 * capturing 24-bit RGB and/or 8-bit Gray-scale image with Sensoray Model
 * 811/911/614/314/611/311, and saving the images into bitmap PPM/PGM files, 
 * using saa7134 (for Model 811/911/614/314) or bttv (for Model 611/311) driver,
 * complying with V4L2 API.
 * 
 * Note: this program is for Linux platform only as application demo program.
 *
 * Author:      
 *              Charlie X. Liu and Pete Eberlein
 *              Sensoray Company, Inc. 2006~2009
 * Revision:
 *              Nov. 20, 2006	Charlie X. Liu	initial, for color image 
 *                                              capturing/grabbing.
 *              Mar. 04, 2009	Pete Eberlein	integrated Gray-scale capturing support, 
 *                                              in this one program.
 *              Dec. 09, 2009   Charlie X. Liu	added capturing support and comments
 *						                        for Sensoary Model 811 and 911
 */

 /***
 *
 * Usage Notes:
 *
 *		Compiling:
 *				gcc [-g] -o sx1xCapV4L2 sx1xCapV4L2.c
 *
 *		Switching:	bewteen 24-bit RGB color image capturing and 8-bit Gray-scale image capturing:
 *                  In the code below, enable "#define BYTESPERPIXEL 3" and comment-out 
 *                  "#define BYTESPERPIXEL 1" for 24-bit RGB color image capturing; Or,
 *				    comment-out "#define BYTESPERPIXEL 3" and enable "#define BYTESPERPIXEL 1" 
 *				    for 8-bit Gray-scale image capturing;
 *
 *		Number of frames:
 *
 *				    specify number of frames to be captured in:  #define GRAB_NUM_FRAMES <number>
 *
 ***/



 // Modyfied by Krishnan Raghavan to suit our requirements Dated 7th AUGUST 2014.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include <getopt.h>             /* getopt_long() */

#include <fcntl.h>              /* low-level i/o */
#include <unistd.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>

#include <asm/types.h>          /* for videodev2.h */

#include <linux/videodev2.h>




#define CLEAR(x) memset (&(x), 0, sizeof (x))

#define WIDTH             640
#define HEIGHT            480
#define GRAB_NUM_FRAMES   1
#define BYTESPERPIXEL     3	/* for color */
//#define BYTESPERPIXEL     1	/* for greyscale */

char filename[60];

typedef enum {
	IO_METHOD_READ,
	IO_METHOD_MMAP,
	IO_METHOD_USERPTR,
} io_method;

struct buffer {
        void *          start;
        size_t          length;
};

static char *           dev_name    = NULL;
static io_method	io	    = IO_METHOD_MMAP;
static int              fd          = -1;
struct buffer *         buffers     = NULL;
static unsigned int     n_buffers   = 0;

typedef struct 
{
    int stride;
    int width;
    int height;
} image_info_t;



static int save_image_uncompressed(const unsigned char *image, const char *szFilename, image_info_t *info, int type);

static void errno_exit (const char * s)
{
        fprintf (stderr, "%s error %d, %s\n",
                 s, errno, strerror (errno));

        exit (EXIT_FAILURE);
}

static int xioctl (int fd,
                   int request,
                   void * arg)
{
        int r;

        do r = ioctl (fd, request, arg);
        while (-1 == r && EINTR == errno);

        return r;
}

static void process_image ( const void * p, int frame )
{
        image_info_t info;
        char szFilebase[60] = filename ;

        //fputc ('.', stdout);
        //fflush (stdout);

        //printf ("frame = %i\tptr = %p\n", frame, p);

        // set up the image save( or if SDL, display to screen)
        info.width  = WIDTH;
        info.height = HEIGHT;
        info.stride = info.width*BYTESPERPIXEL;
        //sprintf( szFilename, "%s%d", szFilebase, frame);

        save_image_uncompressed( p, szFilebase, &info, 
		(BYTESPERPIXEL == 3) ? 1 : 2);

        printf("writing into file %s.p%cm\n", szFilebase,
		(BYTESPERPIXEL == 3) ? 'p' : 'g');
}

static int read_frame ( int count )
{
        struct v4l2_buffer buf;
	unsigned int i;

	switch (io) {
	case IO_METHOD_READ:
    		if (-1 == read (fd, buffers[0].start, buffers[0].length)) {
            		switch (errno) {
            		case EAGAIN:
                    		return 0;

			         case EIO:
				        /* Could ignore EIO, see spec. */

				        /* fall through */

			         default:
				            errno_exit ("read");
			         }
		      }

    		  process_image (buffers[0].start, count);

		      break;

	       }
	       return 1;
}


static void mainloop ( void )
{
	unsigned int count;

        count = GRAB_NUM_FRAMES;

        while (count-- > 0) {
                for (;;) {
                        fd_set fds;
                        struct timeval tv;
                        int r;

                        FD_ZERO (&fds);
                        FD_SET (fd, &fds);

                        /* Timeout. */
                        tv.tv_sec = 2;
                        tv.tv_usec = 0;

                        r = select (fd + 1, &fds, NULL, NULL, &tv);

                        if (-1 == r) {
                                if (EINTR == errno)
                                        continue;

                                errno_exit ("select");
                        }

                        if (0 == r) {
                                fprintf (stderr, "select timeout\n");
                                exit (EXIT_FAILURE);
                        }

			if ( read_frame(GRAB_NUM_FRAMES-count) )
                    		break;
	
			/* EAGAIN - continue select loop. */
            }
        }
}


static void uninit_device ( void )
{
        unsigned int i;

	switch (io) {
	case IO_METHOD_READ:
		free (buffers[0].start);
		break;
	}

	free (buffers);
}


static void init_read ( unsigned int buffer_size )
{
        buffers = calloc (1, sizeof (*buffers));

        if (!buffers) {
                fprintf (stderr, "Out of memory\n");
                exit (EXIT_FAILURE);
        }

	buffers[0].length = buffer_size;
	buffers[0].start = malloc (buffer_size);

	if (!buffers[0].start) {
    		fprintf (stderr, "Out of memory\n");
            	exit (EXIT_FAILURE);
	}
}


static void init_device ( void )
{
        struct v4l2_capability cap;
        struct v4l2_cropcap cropcap;
        struct v4l2_crop crop;
        struct v4l2_format fmt;
	unsigned int min;
	int input, standard;

        printf ("\nstaring device initialization, for %s, ...\n", dev_name);

        if (-1 == xioctl (fd, VIDIOC_QUERYCAP, &cap)) {
                if (EINVAL == errno) {
                        fprintf (stderr, "%s is no V4L2 device\n",
                                 dev_name);
                        exit (EXIT_FAILURE);
                } else {
                        errno_exit ("VIDIOC_QUERYCAP");
                }
        }

        if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
                fprintf (stderr, "%s is no video capture device\n",
                         dev_name);
                exit (EXIT_FAILURE);
        }

	switch (io) {
	case IO_METHOD_READ:
		if (!(cap.capabilities & V4L2_CAP_READWRITE)) {
			fprintf (stderr, "%s does not support read i/o\n",
				 dev_name);
			exit (EXIT_FAILURE);
		}
        break;
	}

        /* Select video input, video standard and tune here. */

        /* Reset Cropping */
        printf ("...reset cropping of %s ...\n", dev_name);
        CLEAR (cropcap);
        cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

        if (-1 == xioctl (fd, VIDIOC_CROPCAP, &cropcap)) {
                /* Errors ignored. */
        }

        crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        crop.c = cropcap.defrect; /* reset to default */

        if (-1 == xioctl (fd, VIDIOC_S_CROP, &crop)) {
                switch (errno) {
                case EINVAL:
                        /* Cropping not supported. */
                        break;
                default:
                        /* Errors ignored. */
                        break;
                }
        }
        usleep(1000);

        /* Select standard */
        printf ("...select standard of %s ...\n", dev_name);
	standard = V4L2_STD_NTSC;
        if (-1 == xioctl (fd, VIDIOC_S_STD, &standard)) {
                perror ("VIDIOC_S_STD");
                exit (EXIT_FAILURE);
        }
        usleep(1000);

        /* Select input */
        printf ("...select input channel of %s ...\n\n", dev_name);
	input = 0;	// Composite-0
        if (-1 == ioctl (fd, VIDIOC_S_INPUT, &input)) {
                perror ("VIDIOC_S_INPUT");
                exit (EXIT_FAILURE);
        }
        usleep(1000);


        CLEAR (fmt);

        fmt.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        fmt.fmt.pix.width       = WIDTH; 
        fmt.fmt.pix.height      = HEIGHT;
        //fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB24; //V4L2_PIX_FMT_YUYV;
	if (BYTESPERPIXEL == 3)
	        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_BGR24;
	else
	        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_GREY;
        fmt.fmt.pix.field       = V4L2_FIELD_INTERLACED;
        //fmt.fmt.pix.field       = V4L2_FIELD_ANY;     //works too

        if (-1 == xioctl (fd, VIDIOC_S_FMT, &fmt))
                errno_exit ("VIDIOC_S_FMT");

        /* Note VIDIOC_S_FMT may change width and height. */

	/* Buggy driver paranoia. */
	min = fmt.fmt.pix.width * BYTESPERPIXEL;
	if (fmt.fmt.pix.bytesperline < min)
		fmt.fmt.pix.bytesperline = min;
	min = fmt.fmt.pix.bytesperline * fmt.fmt.pix.height;
	if (fmt.fmt.pix.sizeimage < min)
		fmt.fmt.pix.sizeimage = min;

	switch (io) {
	case IO_METHOD_READ:
		init_read (fmt.fmt.pix.sizeimage);
		break;
	}
}

static void close_device ( void )
{
        if (-1 == close (fd))
	        errno_exit ("close");

        fd = -1;
}

static void open_device ( void )
{
        struct stat st; 

        if (-1 == stat (dev_name, &st)) {
                fprintf (stderr, "Cannot identify '%s': %d, %s\n",
                         dev_name, errno, strerror (errno));
                exit (EXIT_FAILURE);
        }

        if (!S_ISCHR (st.st_mode)) {
                fprintf (stderr, "%s is no device\n", dev_name);
                exit (EXIT_FAILURE);
        }

        fd = open (dev_name, O_RDWR /* required */ | O_NONBLOCK, 0);

        if (-1 == fd) {
                fprintf (stderr, "Cannot open '%s': %d, %s\n",
                         dev_name, errno, strerror (errno));
                exit (EXIT_FAILURE);
        }
}


/** saves an image to bmp file
 *  @param filename is the full file path to save( minus the extension)
 *  @param *image, pointer to image data in Y Cr Cb format
 *  @param type is the type( 0-- BMP,  1--ppm,  2--pgm)
 *  @return 0 on success, -1 if file invalid
*/
static int save_image_uncompressed(const unsigned char *image, const char *szFilename, image_info_t *info ,int type)
{
    char ppmheader[256];
    char name[60];
    FILE *fptr;
    int val;
    int rlen; // row length or number of columns( unsigned chars== pixels * unsigned chars per pixel)
    int clen; // column length or number of rows
    int bpp;  // unsigned chars per pixel! not bits
    int i,j;

    rlen = info->stride;
    clen = info->height;
    bpp = info->stride / info->width;

    // open the file
    if (type == 1)
        sprintf(name, "%s.%s", szFilename, "ppm");
    else if (type == 2)
        sprintf(name, "%s.%s", szFilename, "pgm");
    else
        sprintf(name, "%s.%s", szFilename, "bmp");

    fptr = fopen( name, "wb");

    if (fptr == NULL)
        return -1;

    if (type == 1)  //for ppm
    {
        //sprintf( ppmheader, "P6\n%d %d\n255\n", info->width, info->height);
        sprintf( ppmheader, "P6\n#ppm image\n%d %d\n255\n", info->width, info->height);
        fwrite( ppmheader, 1, strlen(ppmheader), fptr);
        // write out the rows
        for ( i=0; i<info->height; i++)
        {
            //fwrite( &image[info->stride*i], 1, info->stride, fptr);
            for ( j=0; j<info->width; j++ ) {
            fwrite( &image[info->stride*i+j*3+2], 1, 1, fptr);
            fwrite( &image[info->stride*i+j*3+1], 1, 1, fptr);
            fwrite( &image[info->stride*i+j*3+0], 1, 1, fptr);
            }
        }
    }
    else if (type == 2)  //for pgm
    {
        //sprintf( ppmheader, "P5\n%d %d\n255\n", info->width, info->height);
        sprintf( ppmheader, "P5\n#ppm image\n%d %d\n255\n", info->width, info->height);
        fwrite( ppmheader, 1, strlen(ppmheader), fptr);
        // write out the rows
        for ( i=0; i<info->height; i++)
        {
            //fwrite( &image[info->stride*i], 1, info->stride, fptr);
            for ( j=0; j<info->width; j++ ) {
            fwrite( &image[info->stride*i+j], 1, 1, fptr);
            }
        }
    }
    else // for BMP. needs to be debugged/tested
    {
        val = 0x4d42;                   fwrite(&val,1,2,fptr); //bmp signature
        val = (rlen * clen / bpp) + 54; fwrite(&val,1,4,fptr); //size of bmp file
        val = 0;                        fwrite(&val,1,2,fptr); //must be 0
        val = 0;                        fwrite(&val,1,2,fptr); //must be 0
        val = 54;                       fwrite(&val,1,4,fptr); //offset to image start
        val = 40;                       fwrite(&val,1,4,fptr); //must be 40
        val = rlen / bpp;               fwrite(&val,1,4,fptr); //image width in pixels
        val = clen ;                    fwrite(&val,1,4,fptr); //image height in pixels
        val = 1;                        fwrite(&val,1,2,fptr); //must be 1
        val = bpp * 8;                  fwrite(&val,1,2,fptr); //bits per pixel
        val = 0;                        fwrite(&val,1,4,fptr); //compression (none)
        val = (rlen * clen / bpp);      fwrite(&val,1,4,fptr); //size of image data
        val = 2835;                     fwrite(&val,1,4,fptr); //horizontal pixels/m
        val = 2835;                     fwrite(&val,1,4,fptr); //vertical pixels/m
        val = 0;                        fwrite(&val,1,4,fptr); //colors in image, or 0
        val = 0;                        fwrite(&val,1,4,fptr); //important colors,or 0

#ifndef REVERSE_BMP
        // BMP reverse the image order -- retrieves in reverse order
        for (i= clen-1; i>= 0; i--)  //rows
        {
            fwrite( &image[i*rlen], 1, rlen, fptr);
        }
#else
        // regular BMP order.
        for (i= 0; i<clen; i++)      //rows
        {
            fwrite( &image[i*rlen], 1, rlen, fptr);
        }
#endif

    }

    fclose( fptr);
    return 0;
}



// Main function where the looping starts for getting the images from the board 


void ImagetoSSD(string f)
{     
        strcpy(filename , f) ;
        
        dev_name = "/dev/video0";
        
        io = IO_METHOD_READ;

        open_device ();

        init_device ();

        mainloop ();

        uninit_device ();

        close_device ();

        exit (EXIT_SUCCESS);

        return 0;
}
