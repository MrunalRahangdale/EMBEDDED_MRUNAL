# Embedded - Embedded C Utility Library

*Candidate:* Mrunal 
*Assignment Module:* UART Frame Parser (uart_parser.c)

## Module Overview
A localized, zero-dependency UART byte-stream execution state machine built in standard C99. The module evaluates incoming bytes sequentially and incorporates an explicit delta-time framework to handle lossy inter-byte transmission timeouts cleanly without blocking system routines.

## Compilation and Execution
The project uses strict compiler verification flags and requires standard development tools (gcc).

Compile without warnings:
```bash
gcc -Wall -std=c99 uart_parser.c -o uart_parser
