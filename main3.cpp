#include <iostream>
#include <fstream>
#include <cstdint>
#include <vector>
#include <sys/ioctl.h>
#include <linux/fb.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>

// Define the base address of the register map
#define BASE_ADDR 0xce000000

// Define the register offsets
#define JPEG_CTRL 0x00
#define JPEG_STATUS 0x04
#define JPEG_SRC 0x08
#define JPEG_DST 0x0C

// Define the JPEG_CTRL register bits
enum JPEG_CTRL_Bits {
    START = 31,
    ABORT = 30,
    LENGTH = 23
};

// Function to read a JPEG image file
std::vector<uint8_t> readJpegImage(const std::string& filename) {
    std::ifstream file(filename, std::ios::binary);
    if (!file) {
        std::cerr << "Error opening file: " << filename << std::endl;
        return {};
    }

    file.seekg(0, std::ios::end);
    size_t fileSize = file.tellg();
    file.seekg(0, std::ios::beg);

    std::vector<uint8_t> imageData(fileSize);
    file.read(reinterpret_cast<char*>(imageData.data()), fileSize);

    std::cout << "File opened successfully: " << filename << std::endl;

    return imageData;
}

int main() {
    // Open the memory device
    int mem_fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (mem_fd == -1) {
        perror("Error opening /dev/mem");
        return 1;
    }
    std::cout << "/dev/mem opened successfully" << std::endl;

    // Map the register base address
    void* reg_base = mmap(NULL, getpagesize(), PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd, BASE_ADDR);
    if (reg_base == MAP_FAILED) {
        perror("Error mapping register base address");
        close(mem_fd);
        return 1;
    }
    std::cout << "Register base address mapped successfully" << std::endl;

    volatile uint32_t* jpeg_ctrl = (volatile uint32_t*)((char*)reg_base + JPEG_CTRL);
    volatile uint32_t* jpeg_status = (volatile uint32_t*)((char*)reg_base + JPEG_STATUS);
    volatile uint32_t* jpeg_src = (volatile uint32_t*)((char*)reg_base + JPEG_SRC);
    volatile uint32_t* jpeg_dst = (volatile uint32_t*)((char*)reg_base + JPEG_DST);

    // Open the JPEG image file
    std::string filename = "image.jpg";
    std::vector<uint8_t> imageData = readJpegImage(filename);
    if (imageData.empty()) {
        std::cerr << "Failed to read image data" << std::endl;
        return 1;
    }

    // Initialize the decoder
    uint32_t jpegCtrl = 0;
    jpegCtrl |= (1 << START); // Set the START bit
    jpegCtrl |= (imageData.size() << LENGTH); // Set the LENGTH field

    // Allocate memory for the RGB565 buffer
    uint16_t* rgb565BufferPtr = new uint16_t[imageData.size() / 3 * 2];
    std::cout << "Allocated memory for RGB565 buffer" << std::endl;

    // Set the JPEG_SRC register to the address of the JPEG image data
    uintptr_t jpegSrcAddr = reinterpret_cast<uintptr_t>(&imageData[0]);
    *jpeg_src = static_cast<uint32_t>(jpegSrcAddr);
    std::cout << "Set JPEG_SRC register" << std::endl;

    // Set the JPEG_DST register to the address of the RGB565 buffer
    uintptr_t jpegDstAddr = reinterpret_cast<uintptr_t>(rgb565BufferPtr);
    *jpeg_dst = static_cast<uint32_t>(jpegDstAddr);
    std::cout << "Set JPEG_DST register" << std::endl;

    // Start the decoder
    *jpeg_ctrl = jpegCtrl;
    std::cout << "Started the decoder" << std::endl;

    // Wait for the decoder to finish
    while ((*jpeg_status & 1) == 0) {
        // Busy-wait
    }
    std::cout << "Decoder finished" << std::endl;

    // Open the framebuffer device
    int fbfd = open("/dev/fb0", O_RDWR);
    if (fbfd == -1) {
        perror("Error opening framebuffer device");
        return 1;
    }
    std::cout << "/dev/fb0 opened successfully" << std::endl;

    // Get the framebuffer fixed screen information
    struct fb_fix_screeninfo fb_fix;
    if (ioctl(fbfd, FBIOGET_FSCREENINFO, &fb_fix) == -1) {
        perror("Error getting framebuffer fixed screen information");
        return 1;
    }
    std::cout << "Got framebuffer fixed screen information" << std::endl;

    // Get the framebuffer variable screen information
    struct fb_var_screeninfo fb_var;
    if (ioctl(fbfd, FBIOGET_VSCREENINFO, &fb_var) == -1) {
        perror("Error getting framebuffer variable screen information");
        return 1;
    }
    std::cout << "Got framebuffer variable screen information" << std::endl;

    // Map the framebuffer into memory
    char* fbmem = static_cast<char*>(mmap(NULL, fb_fix.smem_len, PROT_READ | PROT_WRITE, MAP_SHARED, fbfd, 0));
    if (fbmem == MAP_FAILED) {
        perror("Error mapping framebuffer into memory");
        return 1;
    }
    std::cout << "Mapped framebuffer into memory" << std::endl;

    // Copy the RGB565 data into the framebuffer
    uint16_t* rgb565Ptr = reinterpret_cast<uint16_t*>(rgb565BufferPtr); // Use the correct pointer
    for (int y = 0; y < fb_var.yres; y++) {
        for (int x = 0; x < fb_var.xres; x++) {
            uint16_t pixel = rgb565Ptr[y * fb_var.xres + x];
            *((uint16_t*)fbmem + y * fb_fix.line_length / 2 + x) = pixel;
        }
    }
    std::cout << "Copied RGB565 data into framebuffer" << std::endl;

    // Unmap the framebuffer from memory
    munmap(fbmem, fb_fix.smem_len);
    std::cout << "Unmapped framebuffer from memory" << std::endl;

    // Close the framebuffer device
    close(fbfd);
    std::cout << "Closed framebuffer device" << std::endl;

    // Unmap the register base address
    munmap(reg_base, getpagesize());
    std::cout << "Unmapped register base address" << std::endl;

    // Close the memory device
    close(mem_fd);
    std::cout << "Closed /dev/mem" << std::endl;

    return 0;
}
