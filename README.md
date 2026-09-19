# Uart-Comm

STM32 (HAL, DMA) uzerinde byte-byte UART alimi yapan ve karakter kaybini
onlemeyi hedefleyen bir haberlesme algoritmasi. RS485 uzerinden IEC 62056-21
(Kohler AEL.TF.20) elektrik sayaci ile handshake yapip parametre okumaktadir;
okunan degerler ayrica bir Xbee modulu uzerinden iletilmek uzere
hazirlanmaktadir (frame olusturma fonksiyonlari dahildir).

Bu kod sadece USER CODE bloklarina yazilmis parcalari icerir. Calistirmak
icin once STM32CubeMX ile bir proje iskeleti olusturup (USART1/2/4, DMA,
DE/RE kontrol pini) `main.c` dosyasindaki USER CODE bolgelerini buradaki
icerikle degistirmeniz gerekir.

## Bilinen sinirlamalar

- `HAL_UART_RxCpltCallback` icinde bir sonraki byte icin DMA yeniden
  `while(temp == 0){ HAL_UART_Receive_DMA(...); }` seklinde ana dongude
  meslegi ile tekrar tetiklenir; donus degeri kontrol edilmez. Bu, standart
  HAL DMA kullanim seklinden (callback icinde yeniden-arm) farklidir.
- `sendxBee()` icindeki ikinci veri kopyalama blogu, ilkiyle ayni kaynagi
  (`buff[0..3]`) kullanir; ikinci sensor/olcum icin tamamlanmamis bir
  yer tutucudur.

## Anahtar Kelimeler

`stm32` `stm32f7` `hal` `cubemx` `keil-uvision` `uart` `usart` `dma`
`rs485` `half-duplex` `iec62056-21` `iec61107` `elektrik-sayaci`
`meter-reading` `kohler` `xbee` `zigbee` `embedded-c` `gomulu-yazilim`
