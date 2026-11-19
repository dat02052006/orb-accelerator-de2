# ============================================================================
# load_image.tcl - Nap anh grayscale 640x480 vao SRAM qua JTAG-to-Avalon Master
# ============================================================================

# 1. CAU HINH DIA CHI BASE TRONG QSYS (Ban kiem tra cot Base trong Qsys va dien vao day)
set MEM_BASE    0x00000000      ;# Base address cua cong ORB_0.avs_mem
set CTRL_BASE   0x00080000      ;# Base address cua cong ORB_0.avs_ctrl
set IMG_PATH    "test_img.bin"  ;# Duong dan file anh nhi phan 640x480 (307,200 bytes)

set IMG_WIDTH   640
set IMG_HEIGHT  480
set TOTAL_BYTES [expr {$IMG_WIDTH * $IMG_HEIGHT}]       ;# 307,200 bytes
set TOTAL_WORDS [expr {$TOTAL_BYTES / 2}]               ;# 153,600 words (16-bit)

# 2. Mo ket noi toi JTAG-to-Avalon Master
set masters [get_service_paths master]
if {[llength $masters] == 0} {
    puts "ERROR: Khong tim thay JTAG-to-Avalon Master! Kiem tra cap USB-Blaster va file .sof."
    return
}
# Chon master dau tien (neu he thong co nhieu master, doi index [lindex $masters 0] cho phu hop)
set jtag_master [lindex $masters 0]
open_service master $jtag_master
puts "OK: Da ket noi JTAG Master: $jtag_master"

# 3. Doc file anh nhi phan 640x480 (307,200 bytes)
if {![file exists $IMG_PATH]} {
    puts "ERROR: Khong tim thay file '$IMG_PATH'"
    close_service master $jtag_master
    return
}
set fp [open $IMG_PATH rb]
set raw_data [read $fp]
close $fp

if {[string length $raw_data] != $TOTAL_BYTES} {
    puts "ERROR: Kich thuoc file khong dung! Can $TOTAL_BYTES bytes, file co [string length $raw_data] bytes."
    close_service master $jtag_master
    return
}

# Chuyen chuoi byte thanh danh sach gia tri 16-bit Little-Endian (pixel chan o byte thap, pixel le o byte cao)
binary scan $raw_data s* word_list
set u16_list {}
foreach w $word_list {
    lappend u16_list [expr {$w & 0xFFFF}]
}

# 4. Bat che do nap anh: avs_ctrl[0] = 0x0001 (load_mode = 1, load_done = 0)
#    Phan cung se tu dong xoa word_cnt ve 0
master_write_16 $jtag_master $CTRL_BASE [list 0x0001]
puts "Dang nap $TOTAL_WORDS words (640x480) vao SRAM qua JTAG..."

# 5. Day du lieu anh theo tung block (chunk) 4096 words de dat toc do JTAG cao nhat
set CHUNK_SIZE 4096
set start_ms [clock milliseconds]

for {set offset 0} {$offset < $TOTAL_WORDS} {incr offset $CHUNK_SIZE} {
    set end_idx [expr {$offset + $CHUNK_SIZE - 1}]
    if {$end_idx >= $TOTAL_WORDS} {
        set end_idx [expr {$TOTAL_WORDS - 1}]
    }
    set sub_list [lrange $u16_list $offset $end_idx]
    
    # Dia chi byte tren Avalon-MM = MEM_BASE + (word_offset * 2)
    set byte_addr [expr {$MEM_BASE + ($offset * 2)}]
    master_write_16 $jtag_master $byte_addr $sub_list
}

set elapsed_ms [expr {[clock milliseconds] - $start_ms}]
puts "Da truyen xong trong ${elapsed_ms} ms."

# 6. Doc lai thanh ghi word_cnt (offset 1 va 2) de xac nhan phan cung da ghi du SRAM
set cnt_regs [master_read_16 $jtag_master [expr {$CTRL_BASE + 2}] 2]
set cnt_lo   [lindex $cnt_regs 0]
set cnt_hi   [lindex $cnt_regs 1]
set hw_words [expr {(($cnt_hi & 0x3) << 16) | ($cnt_lo & 0xFFFF)}]

puts "Kiem tra phan cung: word_cnt = $hw_words / $TOTAL_WORDS words."
if {$hw_words != $TOTAL_WORDS} {
    puts "WARNING: So word ghi vao SRAM khong khop! Kiem tra lai xung clock hoac dia chi MEM_BASE."
}

# 7. Tat load_mode va bat load_done: avs_ctrl[0] = 0x0002 (load_mode = 0, load_done = 1)
master_write_16 $jtag_master $CTRL_BASE [list 0x0002]
puts "HOAN TAT: Da bat load_done = 1. Nios II da san sang chay ORB!"

close_service master $jtag_master