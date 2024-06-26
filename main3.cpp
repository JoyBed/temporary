#include <iostream>
#include <fstream>
#include <cstdint>
#include <vector>
#include <sys/ioctl.h>
#include <linux/fb.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/shm.h>

// Define the base address of the register map
#define CONFIG_JPEG_HW_BASE 0xce000000

// Define the register offsets
#define JPEG_CTRL         0x0
    #define JPEG_CTRL_START                      31
    #define JPEG_CTRL_START_SHIFT                31
    #define JPEG_CTRL_START_MASK                 0x1

    #define JPEG_CTRL_ABORT                      30
    #define JPEG_CTRL_ABORT_SHIFT                30
    #define JPEG_CTRL_ABORT_MASK                 0x1

    #define JPEG_CTRL_LENGTH_SHIFT               0
    #define JPEG_CTRL_LENGTH_MASK                0xffffff

#define JPEG_STATUS       0x4
    #define JPEG_STATUS_UNDERRUN                 0
    #define JPEG_STATUS_UNDERRUN_SHIFT           0
    #define JPEG_STATUS_UNDERRUN_MASK            0x1

    #define JPEG_STATUS_OVERFLOW                 0
    #define JPEG_STATUS_OVERFLOW_SHIFT           0
    #define JPEG_STATUS_OVERFLOW_MASK            0x1

    #define JPEG_STATUS_BUSY                     0
    #define JPEG_STATUS_BUSY_SHIFT               0
    #define JPEG_STATUS_BUSY_MASK                0x1

#define JPEG_SRC          0x8
    #define JPEG_SRC_ADDR_SHIFT                  0
    #define JPEG_SRC_ADDR_MASK                   0xffffffff

#define JPEG_DST          0xc
    #define JPEG_DST_ADDR_SHIFT                  0
    #define JPEG_DST_ADDR_MASK                   0xffffffff

// Function to read a JPEG image file
std::vector<uint8_t> readJpegImage(const std::string& filename) {
    std::ifstream file(filename, std::ios::binary);
    if (!file.is_open()) {
        printf("Error opening file\n");
        return {};
    }

    file.seekg(0, std::ios::end);
    size_t fileSize = file.tellg();
    file.seekg(0, std::ios::beg);

    std::vector<uint8_t> imageData(fileSize);
    file.read(reinterpret_cast<char*>(imageData.data()), fileSize);

    printf("File opened successfully\n");

    return imageData;
}

int main() {
    // Open the memory device
    int mem_fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (mem_fd == -1) {
        printf("Can not access /dev/mem\n");
        return -1;
    }
    printf("/dev/mem opened successfully\n");

    // Map the register base address
    void* reg_base = mmap(NULL, getpagesize(), PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd, CONFIG_JPEG_HW_BASE);
    if (reg_base == MAP_FAILED) {
        printf("Error mapping register base address\n");
        close(mem_fd);
        return 1;
    }
    printf("Register base address mapped successfully\n");

    volatile uint32_t* jpeg_ctrl = (volatile uint32_t*)((char*)reg_base + JPEG_CTRL);
    volatile uint32_t* jpeg_status = (volatile uint32_t*)((char*)reg_base + JPEG_STATUS);
    volatile uint32_t* jpeg_src = (volatile uint32_t*)((char*)reg_base + JPEG_SRC);
    volatile uint32_t* jpeg_dst = (volatile uint32_t*)((char*)reg_base + JPEG_DST);

    // Open the JPEG image file
    std::string filename = "image.jpg";
    std::vector<uint8_t> imageData = readJpegImage(filename);
    if (imageData.empty()) {
        printf("Failed to read image data\n");
        return -1;
    }

    // Initialize/reset the decoder
    *jpeg_ctrl = (1 << JPEG_CTRL_ABORT_SHIFT); // Set the ABORT bit
    //jpeg_ctrl = (imageData.size() << JPEG_CTRL_LENGTH_SHIFT); // Set the LENGTH field

    // Allocate memory for the RGB565 buffer
    uint16_t* rgb565BufferPtr = new uint16_t[imageData.size() / 3 * 2];
    printf("Allocated memory for RGB565 buffer\n");

    // Create a shared memory segment
    int shm_fd = shmget(IPC_PRIVATE, imageData.size(), IPC_CREAT | 0600);
    if (shm_fd == -1) {
        printf("Error creating shared memory segment\n");
        return -1;
    }

    // Attach the shared memory segment to the process
    void* imagePhysAddr = shmat(shm_fd, NULL, 0);
    if (imagePhysAddr == (void*)-1) {
        printf("Error attaching shared memory segment\n");
        return -1;
    }

    // Copy the image data into the shared memory segment
    std::copy(imageData.begin(), imageData.end(), static_cast<uint8_t*>(imagePhysAddr));

    // Set the JPEG_SRC register to the physical address of the JPEG image data
    uintptr_t jpegSrcAddr = reinterpret_cast<uintptr_t>(imagePhysAddr);
    *jpeg_src = static_cast<uint32_t>(jpegSrcAddr);
    printf("Set JPEG_SRC register\n");
    uint32_t jpegsrcreg = static_cast<uint32_t>(*jpeg_src);
    printf("Original value: %zd\n", imagePhysAddr);
    printf("Value from register: %zd\n", jpegsrcreg);

    // Set the JPEG_DST register to the address of the RGB565 buffer
    uintptr_t jpegDstAddr = reinterpret_cast<uintptr_t>(rgb565BufferPtr);
    *jpeg_dst = static_cast<uint32_t>(jpegDstAddr);
    printf("Set JPEG_DST register\n");
    uint32_t jpegdstreg = static_cast<uint32_t>(*jpeg_dst);
    printf("Original value: %zd\n", jpegDstAddr);
    printf("Value from register: %zd\n", jpegdstreg);

    // Start the decoder
    *jpeg_ctrl = ((1 << JPEG_CTRL_START_SHIFT) | imageData.size()); // Set the START bit and LENGTH field
    printf("Started the decoder\n");

    // Wait for the decoder to finish
    while ((*jpeg_status & (1 << JPEG_STATUS_BUSY_SHIFT))!= 0) {
        // Busy-wait
    }
    printf("Decoder finished\n");

    // Detach the shared memory segment from the process
    shmdt(imagePhysAddr);

    // Remove the shared memory segment
    shmctl(shm_fd, IPC_RMID, NULL);

    // Open the framebuffer device
    int fbfd = open("/dev/fb0", O_RDWR);
    if (fbfd == -1) {
        printf("Error opening framebuffer device\n");
        return -1;
    }
    printf("/dev/fb0 opened successfully\n");

    // Get the framebuffer fixed screen information
    struct fb_fix_screeninfo fb_fix;
    if (ioctl(fbfd, FBIOGET_FSCREENINFO, &fb_fix) == -1) {
        printf("Error getting framebuffer fixed screen information\n");
        return -1;
    }
    printf("Got framebuffer fixed screen information\n");

    // Get the framebuffer variable screen information
    struct fb_var_screeninfo fb_var;
    if (ioctl(fbfd, FBIOGET_VSCREENINFO, &fb_var) == -1) {
        perror("Error getting framebuffer variable screen information\n");
        return 1;
    }
    printf("Got framebuffer variable screen information\n");

    // Map the framebuffer into memory
    char* fbmem = static_cast<char*>(mmap(NULL, fb_fix.smem_len, PROT_READ | PROT_WRITE, MAP_SHARED, fbfd, 0));
    if (fbmem == MAP_FAILED) {
        printf("Error mapping framebuffer into memory\n");
        return 1;
    }
    printf("Mapped framebuffer into memory\n");

    // Copy the RGB565 data into the framebuffer
    uint16_t* rgb565Ptr = reinterpret_cast<uint16_t*>(rgb565BufferPtr); // Use the correct pointer
    for (int y = 0; y < fb_var.yres; y++) {
        for (int x = 0; x < fb_var.xres; x++) {
            uint16_t pixel = rgb565Ptr[y * fb_var.xres + x];
            *((uint16_t*)fbmem + y * fb_fix.line_length / 2 + x) = pixel;
        }
    }
    printf("Copied RGB565 data into framebuffer\n");

    // Unmap the framebuffer from memory
    munmap(fbmem, fb_fix.smem_len);
    printf("Unmapped framebuffer from memory\n");

    // Close the framebuffer device
    close(fbfd);
    printf("Closed framebuffer device\n");

    // Unmap the register base address
    munmap(reg_base, getpagesize());
    printf("Unmapped register base address\n");

    // Close the memory device
    close(mem_fd);
    printf("Closed /dev/mem\n");

    return 0;
}
