See it in action:
https://www.youtube.com/watch?v=yItWi6Zw-CM

# Nucleo-Morse
STM32 implementation of an interactive Morse code terminal. Type regular input and have it translated to Morse code!

## Details
. in Morse is a dit.
- in Morse is a dah.

Uses HAL to set up GPIO LED and active speaker. Able to receive keystrokes through UART interrupts and encode them as Morse with appropriate timing.

Morse characters are encoded as 8-bit values, with bits 7-5 giving the length of the code (number of dits/dahs) and the remaining 5 bits set to 1 for dits, 0 for dahs. If the length is shorter than 5, remaining bits are set to 0 and not interpreted.
Characters with 6 dits/dahs (special characters like + - /) will use bits 7-6 as the length and the remaining 6 bits as the code.
