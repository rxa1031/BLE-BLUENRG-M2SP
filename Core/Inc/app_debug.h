/*
 * app_debug.h
 *
 *  Created on: 22-Dec-2025
 *      Author: Rajeev
 */

#ifndef INC_APP_DEBUG_H_
#define INC_APP_DEBUG_H_

#define __LOG_ENABLE_DEBUG__
#define __LOG_ENABLE_WARN__

#define WINDOWS_NEXT_LINE		"\r\n"
#define LINUX_NEXT_LINE			"\n"
#define _NEXT_LINE_					WINDOWS_NEXT_LINE


/* ============================================================================
 * Logging Helpers and Implementations
 * ============================================================================
 */

/* Print directly to output if the calling macro permits it */
#define LOG_PRINT_STD(stream, prefix, fmt, ...) \
    do { \
        fprintf((stream), prefix fmt _NEXT_LINE_, ##__VA_ARGS__); \
        fflush((stream)); \
    } while (false)

/* Base error logger: always enabled */
#define LOG_ERROR(fmt, ...) \
    LOG_PRINT_STD(stderr, "[ERROR] ", fmt, ##__VA_ARGS__)

/* WARN logger: enabled only if explicitly requested */
#ifdef __LOG_ENABLE_WARN__
    #define LOG_WARN(fmt, ...) \
        LOG_PRINT_STD(stderr, "[WARN] ", fmt, ##__VA_ARGS__)
#else
    #define LOG_WARN(fmt, ...) \
        do {} while (0)
#endif

/* INFO logger: enabled only if explicitly requested */
#ifdef __LOG_ENABLE_INFO__
    #define LOG_INFO(fmt, ...) \
        LOG_PRINT_STD(stdout, "[INFO] ", fmt, ##__VA_ARGS__)
#else
    #define LOG_INFO(fmt, ...) \
        do {} while (0)
#endif

/*
 * DEBUG logger:
 *   - Only compiled when both __DEV__ and __LOG_ENABLE_DEBUG__ are defined.
 *   - Otherwise becomes a no-op.
 */
#if defined(__LOG_ENABLE_DEBUG__)
    #define LOG_DEBUG(fmt, ...) \
        LOG_PRINT_STD(stdout, "[DEBUG] ", fmt, ##__VA_ARGS__)
#else
    #define LOG_DEBUG(fmt, ...) \
        do {} while (0)
#endif

#endif /* INC_APP_DEBUG_H_ */
