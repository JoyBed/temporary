#include <iostream>
#include <fstream>
#include <cstdint>
#include <vector>
#include <sys/ioctl.h>
#include <linux/fb.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>

// Define the register map
enum RegisterMap {
    JPEG_CTRL = 0x00,
    JPEG_STATUS = 0x04,
    JPEG_SRC = 0x08,
    JPEG_DST = 0x0c
};

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

    return imageData;
}

int main() {
    // Open the JPEG image file
    std::string filename = "image.jpg";
    std::vector<uint8_t> imageData = readJpegImage(filename);

    // Initialize the decoder
    uint32_t jpegCtrl = 0;
    jpegCtrl |= (1 << START); // Set the START bit
    jpegCtrl |= (imageData.size() << LENGTH); // Set the LENGTH field

    // Allocate memory for the RGB565 buffer
    uint16_t* rgb565BufferPtr = new uint16_t[imageData.size() / 3 * 2];

    // Set the JPEG_SRC register to the address of the JPEG image data
    uintptr_t jpegSrcAddr = reinterpret_cast<uintptr_t>(&imageData[0]);
    *reinterpret_cast<volatile uint32_t*>(JPEG_SRC) = static_cast<uint32_t>(jpegSrcAddr);

    // Set the JPEG_DST register to the address of the RGB565 buffer
    uintptr_t jpegDstAddr = reinterpret_cast<uintptr_t>(rgb565BufferPtr);
    *reinterpret_cast<volatile uint32_t*>(JPEG_DST) = static_cast<uint32_t>(jpegDstAddr);

    // Start the decoder
    *reinterpret_cast<volatile uint32_t*>(JPEG_CTRL) = jpegCtrl;

    // Wait for the decoder to finish
    while ((*reinterpret_cast<volatile uint32_t*>(JPEG_STATUS) & 1) == 0) {
        // Busy-wait
    }

    // Open the framebuffer device
    int fbfd = open("/dev/fb0", O_RDWR);
    if (fbfd == -1) {
        perror("Error opening framebuffer device");
        return 1;
    }

    // Get the framebuffer fixed screen information
    struct fb_fix_screeninfo fb_fix;
    if (ioctl(fbfd, FBIOGET_FSCREENINFO, &fb_fix) == -1) {
        perror("Error getting framebuffer fixed screen information");
        return 1;
    }

    // Get the framebuffer variable screen information
    struct fb_var_screeninfo fb_var;
    if (ioctl(fbfd, FBIOGET_VSCREENINFO, &fb_var) == -1) {
        perror("Error getting framebuffer variable screen information");
        return 1;
    }

    // Map the framebuffer into memory
    char* fbmem = static_cast<char*>(mmap(NULL, fb_fix.smem_len, PROT_READ | PROT_WRITE, MAP_SHARED, fbfd, 0));

    if (fbmem == MAP_FAILED) {
        perror("Error mapping framebuffer into memory");
        return 1;
    }


    // Copy the RGB565 data into the framebuffer
    uint16_t* rgb565Ptr = reinterpret_cast<uint16_t*>(rgb565BufferPtr); // Use the correct pointer
    for (int y = 0; y < fb_var.yres; y++) {
        for (int x = 0; x < fb_var.xres; x++) {
            uint16_t pixel = rgb565Ptr[y * fb_var.xres + x];
            *((uint16_t*)fbmem + y * fb_fix.line_length / 2 + x) = pixel;
        }
    }

    // Unmap the framebuffer from memory
    munmap(fbmem, fb_fix.smem_len);

    // Close the framebuffer device
    close(fbfd);

    return 0;
}
