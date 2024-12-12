#ifndef MAIN_H
#define MAIN_H

// Definição do LOG
#ifdef ENABLE_DEBUG
#define LOG(tag, fmt, ...) printf("[" tag "] " fmt "\n", ##__VA_ARGS__)
#else
#define LOG(tag, fmt, ...) // Nada
#endif

#endif // MAIN_H
