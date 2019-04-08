# hexdumper
Hexdumper for x86-32 Assembly, NASM, uses Linux system calls


assemble:
nasm -f elf hexdumper.asm
ld -m elf_i386 -o hexdumper hexdumper.o

usage: ./hexdumper filename -a
-a switch is optional and displays corresponding ASCII (only traditional 128 charset)



