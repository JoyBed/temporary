#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <cstddef>
#include <cassert>

#define CONFIG_AV_BUFFERS 4
#define CONFIG_SCREEN_WIDTH 1024
#define CONFIG_SCREEN_HEIGHT 768
#define AV_BUF_DEPTH    2
#define AV_BUF_FPS      24
#define AV_BUF_SRATE    44100
#define AV_AUD_SIZE     (((((AV_BUF_SRATE / AV_BUF_FPS) * 4) + 511) / 512) * 512)

struct link_node {
    struct link_node *next;
    struct link_node *prev;
};

struct link_list {
    struct link_node head;
};

void list_init(struct link_list *list) {
    list->head.next = &list->head;
    list->head.prev = &list->head;
}

void list_insert_last(struct link_list *list, struct link_node *node) {
    node->next = &list->head;
    node->prev = list->head.prev;
    list->head.prev->next = node;
    list->head.prev = node;
}

void list_remove(struct link_list *list, struct link_node *node) {
    node->prev->next = node->next;
    node->next->prev = node->prev;
}

int list_is_empty(struct link_list *list) {
    return list->head.next == &list->head;
}

#define list_entry(ptr, type, member) \
    ((type *)((char *)(ptr) - offsetof(type, member)))

struct av_buffer {
    uint8_t             frame_buffer[CONFIG_SCREEN_WIDTH * CONFIG_SCREEN_HEIGHT * AV_BUF_DEPTH] __attribute__ ((aligned (512)));
    uint8_t             src_data[(CONFIG_SCREEN_WIDTH * CONFIG_SCREEN_HEIGHT * AV_BUF_DEPTH) + AV_AUD_SIZE] __attribute__ ((aligned (512)));
    uint8_t            *src_video;
    uint32_t           *src_audio;
    int                 length;
    int                 audio_length;
    struct link_node    list_entry;
};

static struct av_buffer _buffers[CONFIG_AV_BUFFERS] __attribute__ ((aligned (512)));
static struct link_list _buffer_list;

void avbuf_init(void) {
    int i;
    list_init(&_buffer_list);
    for (i=0;i<CONFIG_AV_BUFFERS;i++)
        list_insert_last(&_buffer_list, &_buffers[i].list_entry);
}

struct av_buffer* avbuf_alloc(void) {
    struct av_buffer* buf;
    struct link_node *node;

    if (list_is_empty(&_buffer_list))
        return 0;

    // csr_clr_irq_enable();

    node = _buffer_list.head.next;
    buf  = list_entry(node, struct av_buffer, list_entry);

    list_remove(&_buffer_list, &buf->list_entry);

    // csr_set_irq_enable();

    buf->length       = 0;
    buf->audio_length = 0;

    return buf;
}

void avbuf_free(struct av_buffer *buf) {
    assert(buf);

    // csr_clr_irq_enable();
    list_insert_last(&_buffer_list, &buf->list_entry);
    // csr_set_irq_enable();
}

static uint8_t *uncached_ptr8(uint8_t *p) {
    return p; // removed the cast to uint32_t, as it's not necessary
}

int main(int argc, char *argv[]) {
    if (argc!= 2) {
        fprintf(stderr, "Usage: %s image.jpg\n", argv[0]);
        return 1;
    }

    avbuf_init();

    FILE *f = fopen(argv[1], "rb");
    if (!f) {
        fprintf(stderr, "Failed to open file %s\n", argv[1]);
        return 1;
    }

    struct av_buffer* buf = avbuf_alloc();
    if (!buf) {
        fprintf(stderr, "Failed to allocate buffer\n");
        fclose(f);
        return 1;
    }

    buf->src_video    = &buf->src_data[0];
    fseek(f, 0, SEEK_END);
    buf->length       = ftell(f);
    fseek(f, 0, SEEK_SET);
    fread(buf->src_video, 1, buf->length, f);
    fclose(f);

    // You can add your display code here
    printf("Loaded JPEG image of size %d bytes\n", buf->length);

    avbuf_free(buf);
    return 0;
}
