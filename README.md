# hexdumper
Hexdumper for x86-32 Assembly, NASM, uses Linux system calls

Will be soon updated to contain a working ASCII output


assemble:
nasm -f elf hexdumper.asm
ld -m elf_i386 -o hexdumper hexdumper.o

usage:
./hexdumper filename
