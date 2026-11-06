#ifndef ROBOT_DATA_ALLOCATOR_H
#define ROBOT_DATA_ALLOCATOR_H

#include <definitions.h>
#include <cstddef>
#include <cstdlib>
#include <cstring>
#include <new>
#if defined(_MSC_VER) || defined(__MINGW32__)
    #include <malloc.h>
#endif

namespace RDL_UTILS{

    // matrix layout used in copying data
    enum class MatrixLayout {
        ColMajor = 0,
        RowMajor = 1
    };
    template<typename T>
    inline void copy_matrix(
        const T* src,
        T* dst,
        size_t rows,
        size_t cols,
        MatrixLayout dst_layout
    )
    {
        if(dst_layout == MatrixLayout::ColMajor)
        {
            for(size_t i = 0; i < rows * cols; ++i)
            {
                dst[i] = src[i];
            }
        }
        else
        {
            // col-major -> row-major
            for(size_t i = 0; i < rows; ++i)
            {
                for(size_t j = 0; j < cols; ++j)
                {
                    dst[i * cols + j] =
                        src[i + j * rows];
                }
            }
        }
    }
    
    inline void* aligned_malloc(size_t alignment, size_t size) {
        if (size == 0) return nullptr;
    #if defined(_MSC_VER) || defined(__MINGW32__)
        return _aligned_malloc(size, alignment);
    #else
        void* ptr = nullptr;
        if (posix_memalign(&ptr, alignment, size) != 0) {
            throw std::bad_alloc();
        }
        return ptr;
    #endif
    }

    inline void aligned_free(void* ptr) {
        if (!ptr) return;
    #if defined(_MSC_VER) || defined(__MINGW32__)
        _aligned_free(ptr);
    #else
        std::free(ptr);
    #endif
    }

    constexpr size_t align_elements(size_t n) {
        return ((n + 7) / 8) * 8;
    }

    constexpr size_t CACHE_LINE_SIZE = 64;
    
    class RobotDataAllocator {
    public:
        
        RobotDataAllocator(size_t dim_pos, size_t dof_dyn) 
            : raw_buffer(nullptr) 
        {
            size_t n_kin = align_elements(dim_pos);
            size_t n_dyn = align_elements(dof_dyn);

            size_t size_M    = align_elements(dof_dyn * dof_dyn);
            size_t size_C    = align_elements(dof_dyn * dof_dyn);
            size_t size_J    = align_elements(6 * dof_dyn);

            size_t total_doubles = 0;
            total_doubles += n_kin;       // q
            total_doubles += n_dyn;       // qd
            total_doubles += n_dyn;       // qdd
            total_doubles += size_M;      // M
            total_doubles += size_C;      // C
            total_doubles += n_dyn;       // g
            total_doubles += n_dyn;       // h
            total_doubles += n_dyn;       // tau
            total_doubles += size_J;      // J

            size_t total_bytes = total_doubles * sizeof(double);

            raw_buffer = static_cast<double*>(aligned_malloc(CACHE_LINE_SIZE, total_bytes));

            if (raw_buffer) {
                std::memset(raw_buffer, 0, total_bytes);

                double* ptr = raw_buffer;

                data.q      = ptr; ptr += n_kin;
                data.qd     = ptr; ptr += n_dyn;
                data.qdd    = ptr; ptr += n_dyn;
                data.M      = ptr; ptr += size_M;
                data.C      = ptr; ptr += size_C;
                data.g      = ptr; ptr += n_dyn;
                data.h      = ptr; ptr += n_dyn;
                data.tau    = ptr; ptr += n_dyn;
                data.J      = ptr; ptr += size_J;


                data.dim_pos = dim_pos;
                data.dof_dyn = dof_dyn;
            }
        }

        ~RobotDataAllocator() {
            if (raw_buffer) {
                aligned_free(raw_buffer);
                raw_buffer = nullptr;
            }
        }

        RobotDataAllocator(const RobotDataAllocator&) = delete;
        RobotDataAllocator& operator=(const RobotDataAllocator&) = delete;

        RobotDataAllocator(RobotDataAllocator&& other) noexcept
            : raw_buffer(other.raw_buffer), data(other.data) 
        {
            other.raw_buffer= nullptr;
            std::memset(&other.data, 0, sizeof(RDL::RobotData));
        }

        RDL::RobotData& get_robot_data() { return data; }
        const RDL::RobotData& get_robot_data() const { return data; }

    private:
        double* raw_buffer;
        RDL::RobotData data;
    };
}

#endif