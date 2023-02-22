# Start debugging with GDB in Command Line Mode

## Introduction
The [GNU GDB debugger](https://www.gnu.org/software/gdb/) is the tool used to interact with the micro-controller on the board.
For low-level operations, it uses the ST-Link tool, dedicated to ST microcontrollers.

You can easily find documentation for GDB, from simple [cheat sheets](https://darkdust.net/files/GDB%20Cheat%20Sheet.pdf), [more extended doc](http://www.yolinux.com/TUTORIALS/GDB-Commands.html), and obviously the [official documentation](https://sourceware.org/gdb/current/onlinedocs/gdb/).

You have to plug your development board, and start in a terminal `st-util`

```sh
$ st-util
st-util 1.4.0
2018-11-23T17:08:25 INFO src/common.c: Loading device parameters....
2018-11-23T17:08:25 INFO src/common.c: Device connected is: F334 device, id 0x10016438
2018-11-23T17:08:25 INFO src/common.c: SRAM size: 0x3000 bytes (12 KiB), Flash: 0x10000 bytes (64 KiB) in pages of 2048 bytes
2018-11-23T17:08:25 INFO src/gdbserver/gdb-server.c: Chip ID is 00000438, Core ID is  2ba01477.
2018-11-23T17:08:25 INFO src/gdbserver/gdb-server.c: Listening at *:4242...
```
The `st-util` tool communicates with the GDB debugger through a socket (port 4242 in this case).

## Basic usage

### Start GDB

To start with the `text user interface` (tui), which is easier to start, with the binary file `test.elf`:

```sh
arm-none-eabi-gdb -tui test.elf
```

> **Note:** Pressing the return key without any command repeats the last command. This is especially interesting when executing programs step by step (commands ```n``` and ```s```, see below) where pressing the return key will advance one instruction or line according to the command.

#### Startup

A gdb script is provided to initiate communication with the target, load the binary file and run until main. The content of the script is (note: lines beginning with a # are comments):

```
#start st-util from another terminal before running gdb:
#arm-none-eabi-gdb -tui test.elf
tar extended-remote :4242
load
break main
#continue (until main)
c
```

To execute this script, we just use the command in the gdb prompt:

```
source init.gdb
```

> **Note:** As is the case with the Unix shell, gdb tries to automatically complete the command or file depending on the context. In the above command, pressing the tab key just after typing an ```i``` completes the filename if ```init.gdb``` is the only file starting with ```i``` in the current directory.

#### Layout

* `layout src`: same as the `-tui` CLI argument. Display src
* `layout asm`: display assembly

The text window is split in 2 parts, the source code file, and the command file. Most useful shortcuts are:

* Ctrl+X O //toggle focus between the 2 windows.
* Ctrl+X 2 //layout with 3 parts, including asm

### Run/stop

Basic commands are:

* `c` (continue) resumes the execution.
* `s` (step) - execute one source code instruction, **step into** functions.
* `n` (next) - execute one line, **step over** functions.
* `si` (step instruction) - execute one assembly language instruction, **step into** functions.
* `ni` (next instruction) - execute one assembly language instruction, **step over** functions.

The execution can be stopped with `Ctrl+C`.

### Where am i

Command ```where``` it displays the line where the execution has reached. For instance, after running the program until you reach main, ```where``` displays this for the blink example:

```
(gdb) where
#0  main () at blink.c:15
```

Thus indicating that the execution has reached line 15 of the blink.c file in the main() function. Note that if you use the src or asm layout, this information is given to you directly.

### Breakpoints

#### declare breakpoint

You can insert a breakpoint using the source code location, the name of a function, or the memory address.

Insert a breakpoint at instruction with memory address 0x8000502:

```
b *0x8000502
```

Insert a breakpoint at line 26 in file `test.c`:

```
b test.c:26
```

Insert a breakpoint at line 6 of the current C file:

```
b 6
```

Insert a breakpoint at function ```main```

```
b main
```

#### list breakpoints

```
info breakpoints
```

#### Remove breakpoint

Remove breakpoint 1 (the id is given from the previous command)

```
delete 1
```

delete all breakpoints

```
delete
```

### Variables and memory

#### View variables

Following commands can be used either with `print` (show one time, can be abbreviated by `p`) or `display` (show each time the program is stopped, can be abbreviated by `disp`):

Print a variable one time. `/x` is for hexadecimal display. Other formats are:  `/o` (octal), `/d` (decimal), `/u` (unsigned decimal), `/t` (binary), `/f` (float), `/a` (address), `/i` (instruction), `/c` (char), `/s` (string) and `/z` (hex, zero padded on the left)

```
print/x i
```

Or from an address it uses a C like syntax with pointer dereferencing):

```
print/x *0x80004d8
```

Access to a structure (as in C):

```
print/x tim->CNT
```

Examine memory: `4` units (i.e. 4 words in this case), `x` means hexa, `w` means that unit is word (4 bytes)

```
x/4xw *0x20002ff8
```

or (get the 4 words above the stack pointer)

```
x/4xw $sp
```

Get the value of the General Purpose Registers (GPR)

```
info registers
```

### Special case for the STM32 peripherals

the peripheral address is computed by the preprocessor from different `#define` commands. To get an access with gdb, we have to declare it in the C file:

```c
volatile GPIO_TypeDef * __attribute__((unused)) gpioa=GPIOA;
```

then, the access is easy in GDB:

```
print/x gpioa->MODER
```

### Update

We use the `set` keyword, example:

```
set gpioa->MODER = gpioa->MODER | 0xa0
```

### Quit

Type the `quit` command to exit GDB. If a program is running, GDB will ask for confirmation.

## Session example

When the board is connected to the USB, `st-util` is started in a terminal.

```
Eridani-5721:blink jlb$ st-util 
st-util 1.3.0
2021-01-06T14:05:14 INFO /Users/jerry/Downloads/stlink-master/src/common.c: Loading device parameters....
2021-01-06T14:05:14 INFO /Users/jerry/Downloads/stlink-master/src/common.c: Device connected is: L43x device, id 0x10016435
2021-01-06T14:05:14 INFO /Users/jerry/Downloads/stlink-master/src/common.c: SRAM size: 0xc000 bytes (48 KiB), Flash: 0x40000 bytes (256 KiB) in pages of 2048 bytes
2021-01-06T14:05:14 INFO /Users/jerry/Downloads/stlink-master/src/gdbserver/gdb-server.c: Chip ID is 00000435, Core ID is  2ba01477.
2021-01-06T14:05:14 INFO /Users/jerry/Downloads/stlink-master/src/gdbserver/gdb-server.c: Listening at *:4242...
```

> **Note:** `Eridani-5721:blink jlb$ ` is the command prompt for my terminal. `Eridani-5721` is the name of my computer, `blink` is the current directory and `jlb` is my user name.

In another terminal (here we take the blink example of Trampoline), we launch GDB.

```
Eridani-5721:blink jlb$ arm-none-eabi-gdb -tui blink_exe
```

Then, when the GDB prompt appears, `(gdb)`, you type:

```
(gdb) source init.gdb
```

The commands of `init.gdb' are executed, GDB connects via `st-util' to the board (command `tar extended-remote :4242`), the program is loaded (command `load`) and a breakpoint is set on the main function (command `break main`).

The execution is then at the first instruction of the reset handler.

You can then type `c` so that the execution continues until a breakpoint is encountered or the user types `Ctrl-C`.

```
(gdb) c
```

The execution stops at the breakpoint of main:

```
Breakpoint 1, main () at blink.c:15
```

You can request to display PC each time the execution stops:

```
(gdb) disp/x $pc
1: /x $pc = 0x8001e98
```

Let's now set a breakpoint on the function of the blink task.

```
(gdb) b blink_function 
Breakpoint 2 at 0x8001eb8: file blink.c, line 22.
```

And let's continue the execution by typing `c`, it stops almost immediately on the breakpoint we've just positioned.

```
(gdb) c
Continuing.

Breakpoint 2, blink_function () at blink.c:22
1: /x $pc = 0x8001eb8
```

The stack pointer register can now be examined:

```
(gdb) p/x $sp
$1 = 0x20000524
```

Now we can execute the program step by step at the assembler instruction level by typing `si` the first time and then the return key to repeat this last command:

```
(gdb) si
1: /x $pc = 0x8001eba
1: /x $pc = 0x8001ebc
1: /x $pc = 0x8001ebe
1: /x $pc = 0x8001ec2
1: /x $pc = 0x8001ec4
TerminateTask () at blink/tpl_invoque.S:470
1: /x $pc = 0x8001cb0
1: /x $pc = 0x8001cb4
1: /x $pc = 0x8001cb6
1: /x $pc = 0x8001cb8
1: /x $pc = 0x8001cba
tpl_sc_handler () at ../../../../../../machines/cortex/tpl_sc_handler.S:128
1: /x $pc = 0x80019b8
(gdb) 
```
