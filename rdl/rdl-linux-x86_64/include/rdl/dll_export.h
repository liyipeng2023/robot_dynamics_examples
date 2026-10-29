#ifndef RDL_DLL_EXPORT
#define RDL_DLL_EXPORT

#ifdef _WIN32
    #ifdef RDL_DLL_EXPORT_ENABLE
        #define RDL_EXPORT __declspec( dllexport )
    #else
        #define RDL_EXPORT __declspec( dllimport )
    #endif
#elif defined(__GNUC__) || defined(__clang__)
    // GCC/Clang 
    #define RDL_EXPORT __attribute__((visibility("default")))
#else
    #define RDL_EXPORT 
#endif
#endif