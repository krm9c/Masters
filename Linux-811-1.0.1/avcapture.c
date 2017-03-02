/*
 *  Based on free capture.c From V4L2 website
 *  Based on output-example.c from ffmpeg/libavformat
 *  Modified by Sensoray Company Copyright 2011
 *  for raw AVI capture with A/V sync from 811 board
 *
 *  Ubuntu:
 *  su -c "apt-get install libavformat-dev libasound2-dev"    (one time)
 *  make avcapture LDLIBS="-lavformat -lavcodec -lavutil -lasound"
 * 
 *  RedHat/Fedora:
 *  su -c "yum install ffmpeg-devel"       (one time)
 *  make avcapture LDLIBS="-lavformat -lavcodec -lavutil -lasound -lpthread" CFLAGS="-I/usr/include/ffmpeg"
 *
**/

/* choose one: */
//#define USE_REALTIME_TIMESTAMPS	// uses gettimeofday
#define USE_MONOTONIC_TIMESTAMPS	// uses clock_gettime(CLOCK_MONONTONIC_RAW
//#define USE_RELATIVE_TIMESTAMPS	// uses gettimeofday relative to videobuf.timestamp
//#define USE_ALSA_TIMESTAMPS		// uses snd_pcm_status_get_timestamp

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <getopt.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <malloc.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <asm/types.h>
#include <pthread.h>
#include <time.h>
#include <signal.h>

#ifndef CLOCK_MONOTONIC_RAW
#define CLOCK_MONOTONIC_RAW CLOCK_MONOTONIC
#endif

#include <linux/videodev2.h>
#include <libavformat/avformat.h>
#include <alsa/asoundlib.h>

#define do_ioctl(fd, cmd, arg) do {	\
	while (ioctl(fd, cmd, arg) < 0) {\
		if (errno == EAGAIN)	\
			continue;	\
		perror(#cmd);		\
		exit(EXIT_FAILURE);	\
	}				\
} while (0)

#define error_if(cond, msg...) do {	\
	if (cond) {			\
		fprintf(stderr, msg);	\
		exit(EXIT_FAILURE);	\
	}				\
} while (0)


/* V4L2 global variables */
int			fd;
char			*dev_name;
struct v4l2_format	fmt = { .type = V4L2_BUF_TYPE_VIDEO_CAPTURE };
struct v4l2_capability	cap;
struct v4l2_streamparm	parm = { .type = V4L2_BUF_TYPE_VIDEO_CAPTURE };
struct buffer {
	void *start;
	size_t length;
}			*buffers = NULL;
unsigned int		n_buffers;
int			video_card_num = 0;

/* ALSA global variables */
unsigned int		pcm_rate = 32000;
snd_pcm_t		*pcm = NULL;
snd_pcm_status_t	*pcm_status = NULL;
snd_pcm_hw_params_t	*hwparams = NULL;
snd_pcm_sw_params_t	*swparams = NULL;

/* AVFormat global variables */
char			*out_name;
AVOutputFormat		*av_fmt;
AVStream		*vid_st;
AVStream		*aud_st;
AVFormatContext		*av_ctx;

/* global list variables */
pthread_cond_t av_cond = PTHREAD_COND_INITIALIZER;
pthread_mutex_t av_lock = PTHREAD_MUTEX_INITIALIZER;

AVPacketList *packet_list = NULL;
AVPacketList *video_free_list = NULL;
AVPacketList *audio_free_list = NULL;
AVPacketList packet_done; // special packet to signal stream done

/* these are needed by av_thread_func */
static unsigned int clip_counts = 0;

/* global quit flag */
static int quit = 0;




static int determine_card_num(void)
{
	struct v4l2_capability cap2;
	char *num = dev_name + strlen("/dev/video");
	int n = atoi(num);
	int fd2;
	int card = 1;
	
	while (n-- > 0) {
		sprintf(num, "%d", n);
		fd2 = open(dev_name, O_RDWR);
		if (fd2 < 0)
			continue;
		do_ioctl(fd2, VIDIOC_QUERYCAP, &cap2);
		if (strcmp((char *)cap2.driver, (char *)cap.driver) == 0)
			card++;
		//fprintf(stderr, "%s %d <-> %s %d\n", cap2.driver, n, cap.driver, card);
		close(fd2);
	}
	return card;
}

void init_video(void)
{
	struct v4l2_requestbuffers req = {
		.count	= 4,
		.type	= V4L2_BUF_TYPE_VIDEO_CAPTURE,
		.memory	= V4L2_MEMORY_MMAP,
	};
	v4l2_std_id std;

	fd = open(dev_name, O_RDWR);
	error_if(fd < 0, "%s: %s\n", dev_name, strerror(errno));
	do_ioctl(fd, VIDIOC_QUERYCAP, &cap);
	usleep(1000);
	error_if(!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE) ||
		 !(cap.capabilities & V4L2_CAP_STREAMING),
		 "%s doesn't support streaming video capture\n", dev_name);
	do_ioctl(fd, VIDIOC_G_STD, &std);
	do_ioctl(fd, VIDIOC_G_FMT, &fmt);

	//force to NTSC for the use in US
	std = V4L2_STD_NTSC;
	do_ioctl(fd, VIDIOC_S_STD, &std);
	usleep(1000);

	fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_UYVY;
	fmt.fmt.pix.width = 704;
	fmt.fmt.pix.height = (std & V4L2_STD_525_60) ? 480 : 576;
	do_ioctl(fd, VIDIOC_S_FMT, &fmt);
	usleep(1000);

	do_ioctl(fd, VIDIOC_G_PARM, &parm);
	if (!parm.parm.capture.timeperframe.denominator || !parm.parm.capture.timeperframe.numerator) {
		// guess timeperframe from the video standard
		if (std & V4L2_STD_525_60) {
			parm.parm.capture.timeperframe.denominator = 30000;
			parm.parm.capture.timeperframe.numerator = 1001;
		} else if (V4L2_STD_625_50) {
			parm.parm.capture.timeperframe.denominator = 25;
			parm.parm.capture.timeperframe.numerator = 1;
		}
	}
	do_ioctl(fd, VIDIOC_REQBUFS, &req);
	error_if(req.count < 2, "Insufficient buffer memory on %s\n", dev_name);

	buffers = calloc(req.count, sizeof(*buffers));
	error_if(!buffers, "Out of memory\n");

	for (n_buffers = 0; n_buffers < req.count; ++n_buffers) {
		struct v4l2_buffer buf = {
			.type	= V4L2_BUF_TYPE_VIDEO_CAPTURE,
			.memory	= V4L2_MEMORY_MMAP,
			.index	= n_buffers,
		};
		do_ioctl(fd, VIDIOC_QUERYBUF, &buf);
		buffers[n_buffers].length = buf.length;
		buffers[n_buffers].start =
			mmap(NULL /* start anywhere */,
			     buf.length,
			     PROT_READ | PROT_WRITE /* required */,
			     MAP_SHARED /* recommended */,
			     fd, buf.m.offset);
		error_if(MAP_FAILED == buffers[n_buffers].start,
			"mmap failed: %s\n", strerror(errno));
	}
	video_card_num = determine_card_num();
	//printf("%s: card %d bufs: %d\n", cap.driver, video_card_num, n_buffers);
}

void start_video(void)
{
	unsigned int i;
	for (i = 0; i < n_buffers; ++i) {
		struct v4l2_buffer buf = {
			.type	= V4L2_BUF_TYPE_VIDEO_CAPTURE,
			.memory	= V4L2_MEMORY_MMAP,
			.index	= i,
		};
		do_ioctl(fd, VIDIOC_QBUF, &buf);
	}
	do_ioctl(fd, VIDIOC_STREAMON, &fmt.type);
}

void close_video(void)
{
	unsigned int i;
	do_ioctl(fd, VIDIOC_STREAMOFF, &fmt.type);
	for (i = 0; i < n_buffers; i++)
		error_if(-1 == munmap(buffers[i].start, buffers[i].length),
			"munmap: %s\n", strerror(errno));
	close(fd);
}

void close_audio(void)
{
	if (pcm) {
		snd_pcm_drop(pcm);
		snd_pcm_close(pcm);
	}
	if (pcm_status)
		snd_pcm_status_free(pcm_status);
	if (hwparams)
		snd_pcm_hw_params_free(hwparams);
	if (swparams)
		snd_pcm_sw_params_free(swparams);
	pcm = NULL;
	pcm_status = NULL;
	hwparams = NULL;
	swparams = NULL;
}

void init_audio(void)
{
	int i;
	int rc;
	snd_pcm_info_t *info = NULL;
	char hw_name[10]; 	//"hw:CARD,DEVICE"  e.g. "hw:3,0"
	int card = 0;
	snd_ctl_t *ctl;
	snd_ctl_card_info_t *hw_info;

	rc = snd_pcm_info_malloc(&info);
	if (rc)
		goto cleanup;
	rc = snd_pcm_status_malloc(&pcm_status);
	if (rc)
		goto cleanup;
	rc = snd_ctl_card_info_malloc(&hw_info);
	if (rc)
		goto cleanup;
	
	for(i = 0; card < video_card_num; ++i) {
		sprintf(hw_name, "hw:%d", i);
		error_if(snd_ctl_open(&ctl, hw_name, 0) < 0, "failed to probe sound card\n");
		error_if(snd_ctl_card_info(ctl, hw_info) < 0, "failed to get sound card info\n");
		//printf("card %d name: %s  %s\n", i, snd_ctl_card_info_get_driver(hw_info), cap.driver);
		if (strcasecmp((char *)cap.driver, snd_ctl_card_info_get_driver(hw_info)) == 0)
			++card;
		snd_ctl_close(ctl);
		
	}
	snd_ctl_card_info_free(hw_info);
	strcat(hw_name, ",0");
	rc = snd_pcm_open(&pcm, hw_name, SND_PCM_STREAM_CAPTURE, SND_PCM_NONBLOCK);
	if (rc) {
		printf("%s: snd_pcm_open(%s..) error %d: %s\n", __func__, hw_name, rc, snd_strerror(rc));
		goto cleanup;
	}
	rc = snd_pcm_info(pcm, info);
	if (rc) {
		printf("%s: snd_pcm_info error %d: %s\n", __func__, rc, snd_strerror(rc));
		goto cleanup;
	}
	if (pcm) {
		snd_pcm_uframes_t period = 256;

		//printf("%s: found pcm %s %s\n", __func__, snd_pcm_name(pcm), snd_pcm_info_get_id(info));
			
		rc = snd_pcm_hw_params_malloc(&hwparams);
		if (rc)
			goto cleanup;
		rc = snd_pcm_sw_params_malloc(&swparams);
		if (rc)
			goto cleanup;

		rc = snd_pcm_hw_params_any(pcm, hwparams);
		if (rc)
			goto cleanup;
		rc = snd_pcm_hw_params_set_rate_resample(pcm, hwparams, 1);
		if (rc)
			goto cleanup;
		rc = snd_pcm_hw_params_set_access(pcm, hwparams, SND_PCM_ACCESS_RW_INTERLEAVED);
		if (rc)
			goto cleanup;
		rc = snd_pcm_hw_params_set_format(pcm, hwparams, SND_PCM_FORMAT_S16_LE);
		if (rc)
			goto cleanup;
		rc = snd_pcm_hw_params_set_channels(pcm, hwparams, 2);
		if (rc)
			goto cleanup;
		rc = snd_pcm_hw_params_set_rate_near(pcm, hwparams, &pcm_rate, NULL);
		if (rc)
			goto cleanup;
		rc = snd_pcm_hw_params_set_period_size_near(pcm, hwparams, &period, NULL);
		if (rc)
			goto cleanup;
		rc = snd_pcm_hw_params(pcm, hwparams);
		if (rc)
			goto cleanup;
			
		rc = snd_pcm_sw_params_current(pcm, swparams);
		if (rc)
			goto cleanup;
		rc = snd_pcm_sw_params(pcm, swparams);
		if (rc) {
cleanup:
			printf("%s: pcm config error %d: %s\n", __func__, rc, snd_strerror(rc));
			close_audio();
		}
	}
	snd_pcm_info_free(info);
}

void start_audio(void)
{
	if (pcm) 
		snd_pcm_start(pcm);
}


int map_pix_fmt(void)
{
	switch (fmt.fmt.pix.pixelformat) {
	case V4L2_PIX_FMT_YUYV: return PIX_FMT_YUYV422;
	case V4L2_PIX_FMT_UYVY: return PIX_FMT_UYVY422;
	default: return PIX_FMT_NONE;
	}
}

void init_av_out(void)
{
	AVCodec *codec;
	AVCodecContext *c;
	
	av_register_all();
	
	av_fmt = av_guess_format(NULL, out_name, NULL);
	error_if(!av_fmt, "Error guessing format for %s\n", out_name);
	av_ctx = avformat_alloc_context();
	error_if(!av_ctx, "Error allocating av context\n");
	av_ctx->oformat = av_fmt;
	snprintf(av_ctx->filename, sizeof(av_ctx->filename), "%s", out_name);
	
	// add video stream
	vid_st = av_new_stream(av_ctx, 0);
	error_if(!vid_st, "Error creating av stream\n");
	vid_st->stream_copy = 1;
	c = vid_st->codec;
	c->codec_id = CODEC_ID_RAWVIDEO;
	c->codec_type = AVMEDIA_TYPE_VIDEO;
	c->pix_fmt = map_pix_fmt();
	c->width = fmt.fmt.pix.width;
	c->height = fmt.fmt.pix.height;
	c->time_base.den = parm.parm.capture.timeperframe.denominator;
	c->time_base.num = parm.parm.capture.timeperframe.numerator;
	if (av_ctx->oformat->flags & AVFMT_GLOBALHEADER)
		c->flags |= CODEC_FLAG_GLOBAL_HEADER;

	// add audio stream
	if (pcm) {
		aud_st = av_new_stream(av_ctx, 0);
		error_if(!aud_st, "Error creating av stream\n");
		aud_st->stream_copy = 1;
		c = aud_st->codec;
		c->codec_id = CODEC_ID_PCM_S16LE;
		c->codec_type = AVMEDIA_TYPE_AUDIO;
		c->sample_fmt = SAMPLE_FMT_S16;
		c->bit_rate = pcm_rate * 32;
		c->sample_rate = pcm_rate;
		c->channels = 2;
		if (av_ctx->oformat->flags & AVFMT_GLOBALHEADER)
			c->flags |= CODEC_FLAG_GLOBAL_HEADER;
	}

#ifndef FF_API_FORMAT_PARAMETERS
	error_if(av_set_parameters(av_ctx, NULL) < 0, "Error setting av parameters\n");
#endif
	codec = avcodec_find_encoder(vid_st->codec->codec_id);
	error_if(!codec, "Error finding video encoder\n");
	error_if(avcodec_open(vid_st->codec, codec) < 0, "Error opening video encoder\n");
	
	if (aud_st) {
		codec = avcodec_find_encoder(aud_st->codec->codec_id);
		error_if(!codec, "Error finding audio encoder\n");
		error_if(avcodec_open(aud_st->codec, codec) < 0, "Error opening audio encoder\n");
	}
#ifdef FF_API_DUMP_FORMAT
	av_dump_format(av_ctx, 0, out_name, 1);
#else
	dump_format(av_ctx, 0, out_name, 1);
#endif
#ifdef FF_API_OLD_AVIO
	error_if(avio_open(&av_ctx->pb, out_name, AVIO_FLAG_WRITE) < 0, 
#else
	error_if(url_fopen(&av_ctx->pb, out_name, URL_WRONLY) < 0, 
#endif
		"Error opening %s\n", out_name);
#ifdef FF_API_FORMAT_PARAMETERS
	avformat_write_header(av_ctx, NULL);
#else
	av_write_header(av_ctx);
#endif
}


void close_av_out(void)
{
	unsigned int i;
	if (av_ctx) {
		av_write_trailer(av_ctx);
		if (vid_st) {
			avcodec_close(vid_st->codec);
		}
		for (i = 0; i < av_ctx->nb_streams; i++) {
			av_freep(&av_ctx->streams[i]->codec);
			av_freep(&av_ctx->streams[i]);
		}
#ifdef FF_API_OLD_AVIO
		avio_close(av_ctx->pb);
#else
		url_fclose(av_ctx->pb);
#endif
		av_free(av_ctx);
	}
}



/* write av samples in a separate thread to prevent blocking writes from dropping frames */
static void *av_thread_func(void *arg)
{
	AVPacketList	*pkt = NULL;
	char		*out_name_base = out_name;
	unsigned int 	counts;
	int 		out_name_len;

	out_name_len = strlen(out_name_base) - 4; // all except .avi	
	out_name = (char*) malloc (out_name_len + 10); // adds _NNNN.avi
	error_if(!out_name, "out of memory\n");
	
	for (counts = 0; counts < clip_counts; counts++) {

		sprintf( out_name, "%.*s_%.4d%s", out_name_len, out_name_base, counts, out_name_base + out_name_len);
		printf("\n--- Start A/V capturing and Save the data into file \"%s\" ... \n", out_name);

		init_av_out();
		error_if(vid_st->index != 0 || aud_st->index != 1,
			"incorrect stream indexes video:%d audio:%d\n", vid_st->index, aud_st->index);

		pthread_mutex_lock(&av_lock);
		while (!quit) {
			pkt = packet_list;
			if (!pkt) {
				pthread_cond_wait(&av_cond, &av_lock);
				continue;
			}
			packet_list = pkt->next;
			if (pkt == &packet_done)
				break;  // done

			pthread_mutex_unlock(&av_lock);
			/* this can block if the OS gets busy flushing the write page cache */
			av_interleaved_write_frame(av_ctx, &pkt->pkt);
			pthread_mutex_lock(&av_lock);

			if (pkt->pkt.stream_index == vid_st->index) {
				pkt->next = video_free_list;
				video_free_list = pkt;
			} else {
				pkt->next = audio_free_list;
				audio_free_list = pkt;
			}
		}
		pthread_mutex_unlock(&av_lock);
		close_av_out();
		if (quit)
			break;
	}
	quit = 1; // quit main thread when all clips are done
	free(out_name);
	out_name = out_name_base;
	return NULL;
}

static void append_packet(AVPacketList **list, AVPacket *pkt)
{
	pthread_mutex_lock(&av_lock);
	while (*list) {
		if ((AVPacketList *)pkt == *list)
			printf("packet %p is already on the list!\n", pkt);
		list = &(*list)->next;
	}
	*list = (AVPacketList *)pkt;
	(*list)->next = NULL;
	pthread_cond_signal(&av_cond);
	pthread_mutex_unlock(&av_lock);
}

static AVPacket *get_packet(AVPacketList **free_list)
{
	AVPacketList *pkt;
	
	pthread_mutex_lock(&av_lock);
	pkt = *free_list;
	if (pkt)
		*free_list = pkt->next;
	pthread_mutex_unlock(&av_lock);
	
	if (!pkt) {
		pkt = malloc(sizeof(*pkt));
		pkt->pkt.data = NULL;
	}
	pkt->next = NULL;
	return &pkt->pkt;
}



void quit_handler(int sig)
{
	quit = 1;
}

#define AUDIO_MAX_SIZE 65536

int main(int argc, char *argv[])
{
#ifdef USE_MONOTONIC_TIMESTAMPS
	struct timespec start_ts;
	struct timespec ts;
#else
	struct timeval start_ts;
	struct timeval ts;
#endif
	pthread_t av_thread;
	unsigned int clip_length = 600;		// in seconds, default = 10 minutes
	unsigned long duration = 3600;	// in seconds, default = 1 hour
	unsigned int video_frame_count = 0;
	unsigned int frame_limit;
	struct {
		unsigned int num, den;
	} video_time_base, audio_time_base;
	unsigned int codec_id;

	
	error_if(argc < 3,
		"Usage: %s <video_device> <output_file> [duration] [clip_length]\n"
		" video_device   V4L2 device, ex: /dev/video0\n"
		" output_file    filename, ex: output.avi\n"
		" duration       optional duration in frames, or with a time suffix,\n"
		"                ex: 30s 60m 2h \n"
		" clip_length    optional clip_length in seconds, or with a time suffix,\n"
		"                ex: 20s 10m 1h \n"
		, argv[0]);
	
	dev_name = argv[1];
	out_name = argv[2];

	if (argc > 3) {
		duration = atoi(argv[3]);
		switch (tolower(argv[3][strlen(argv[3])-1])) {
		case 'h':
			duration *= 60;		// minutes
		case 'm':
			duration *= 60;		// seconds
		case 's':
			break;
		default:
			break;
		}
	}
	clip_length = duration; // default clip length is same as duration
	if (argc > 4) {
		clip_length = atoi(argv[4]);
		switch (tolower(argv[4][strlen(argv[4])-1])) {
		case 'h':
			clip_length *= 60;		// minutes
		case 'm':
			clip_length *= 60;		// seconds
		case 's':
			break;
		default:
			break;
		}
	}

	clip_counts = duration/clip_length;
	if (duration != clip_counts * clip_length) clip_counts += 1;
	printf("\nDuration = %lu sec\n", duration);
	printf("Clip_length = %d sec\n", clip_length);
	printf("Clip_counts = %d\n", clip_counts); //exit(0);

	init_video();
	init_audio();

	/* record until ctrl-c is pressed */
	signal(SIGINT, quit_handler);
	signal(SIGTERM, quit_handler);


	start_video();
	start_audio();

	frame_limit = clip_length * parm.parm.capture.timeperframe.denominator
				  / parm.parm.capture.timeperframe.numerator;

	error_if(pthread_create(&av_thread, NULL, av_thread_func, NULL) < 0,
		"failed to create AV thread\n");
	
	while (vid_st == NULL || aud_st == NULL)
		usleep(10);
	/* vid_st and aud_st may disappear during file switch, so save these values */
	video_time_base.num = vid_st->time_base.num;
	video_time_base.den = vid_st->time_base.den;
	audio_time_base.num = aud_st->time_base.num;
	audio_time_base.den = aud_st->time_base.den;
	codec_id = vid_st->codec->codec_id;

	//printf("video time_base %d %d\n", vid_st->time_base.num, vid_st->time_base.den);
	//printf("audio time_base %d %d\n", aud_st->time_base.num, aud_st->time_base.den);

	while (!quit) {
		struct v4l2_buffer buf = {
			.type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
			.memory = V4L2_MEMORY_MMAP,
		};
		AVPacket *pkt = NULL;

		do_ioctl(fd, VIDIOC_DQBUF, &buf);
		if (quit)
			break;
		if (buf.sequence % 30 == 0) {
			fprintf(stderr, ".");
			fflush(stderr);
		}
		if (video_frame_count == 0) {
			/* record first frame timestamp */
#ifdef USE_MONOTONIC_TIMESTAMPS
			clock_gettime(CLOCK_MONOTONIC_RAW, &start_ts);
#else
#ifdef USE_ALSA_TIMESTAMPS
			snd_pcm_status(pcm, pcm_status);
			snd_pcm_status_get_tstamp(pcm_status, &start_ts);
#else
			gettimeofday(&start_ts, NULL);
#endif
#endif
		}

		pkt = get_packet(&video_free_list);
		av_init_packet(pkt);

		if (buf.flags & V4L2_BUF_FLAG_KEYFRAME || codec_id == CODEC_ID_RAWVIDEO)
			pkt->flags |= AV_PKT_FLAG_KEY;
		if (!pkt->data)
			pkt->data = malloc(buffers[0].length);
		memcpy(pkt->data, buffers[buf.index].start, buf.bytesused);
		pkt->size = buf.bytesused;
		pkt->stream_index = 0;
		do_ioctl(fd, VIDIOC_QBUF, &buf);
		
		append_packet(&packet_list, pkt);
		pkt = NULL;
		
		if (pcm) {
			snd_pcm_sframes_t avail;
			//snd_pcm_sframes_t delay;
			int rc;

			pkt = get_packet(&audio_free_list);
			if (!pkt->data)
				pkt->data = malloc(AUDIO_MAX_SIZE);
			av_init_packet(pkt);

#ifdef USE_MONOTONIC_TIMESTAMPS			
			clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
#else
			gettimeofday(&ts, NULL); /* this assumes video driver gets video timestamp using gettimeofday */
#endif
			avail = snd_pcm_avail_update(pcm);
			if (avail <= 0)
				printf("avail=%ld\n", avail);
			//rc = snd_pcm_avail_delay(pcm, &avail, &delay);
			//if (rc < 0)
			//	printf("snd_pcm_avail_delay: %s\n", snd_strerror(rc));
#ifdef USE_ALSA_TIMESTAMPS
			snd_pcm_status(pcm, pcm_status);
			snd_pcm_status_get_tstamp(pcm_status, &ts);
#endif

			//printf("snd ts %llu %u\n", ts.tv_sec, ts.tv_usec);
#ifdef USE_RELATIVE_TIMESTAMPS
			/* pts = (computed video timestamp) + (difference between audio and video timestamp) - (audio sample duration) */
			pkt->pts = (int64_t)video_frame_count * video_time_base.num * audio_time_base.den / (video_time_base.den * audio_time_base.num) +
				   (int64_t)(ts.tv_sec - buf.timestamp.tv_sec) * audio_time_base.den / audio_time_base.num + 
				   (int64_t)(ts.tv_usec - buf.timestamp.tv_usec) * audio_time_base.den / (audio_time_base.num * 1000000);
#else
#ifdef USE_MONOTONIC_TIMESTAMPS
			pkt->pts = (int64_t)(ts.tv_sec - start_ts.tv_sec) * audio_time_base.den / audio_time_base.num + 
			           (int64_t)(ts.tv_nsec - start_ts.tv_nsec) * audio_time_base.den / (audio_time_base.num * 1000000000);
#else
			pkt->pts = (int64_t)(ts.tv_sec - start_ts.tv_sec) * audio_time_base.den / audio_time_base.num + 
			           (int64_t)(ts.tv_usec - start_ts.tv_usec) * audio_time_base.den / (audio_time_base.num * 1000000);
#endif
#endif
			pkt->pts -= (int64_t)avail * audio_time_base.den / (pcm_rate * audio_time_base.num); // adjust by available frames

			if (avail > AUDIO_MAX_SIZE / 4)
				avail = AUDIO_MAX_SIZE / 4;
			rc = snd_pcm_readi(pcm, pkt->data, avail);
			if (rc > 0) {
				pkt->size = rc * 4;
				pkt->flags |= AV_PKT_FLAG_KEY;
				pkt->stream_index = 1;
				append_packet(&packet_list, pkt);
			} else {
				if (avail > 0)
					printf("snd_pcm_readi rc=%d\n", rc);
				append_packet(&audio_free_list, pkt);
			}
			pkt = NULL;
		}
		if (++video_frame_count > frame_limit) {
			video_frame_count = 0;
			append_packet(&packet_list, &packet_done.pkt);
		}
	}
	fprintf(stderr, "\n");
	pthread_cond_signal(&av_cond);
	pthread_join(av_thread, NULL);

	close_video();
	close_audio();
	return 0;

}

