# Uart-Comm

STM32 (HAL, DMA) üzerinde byte-byte UART alımı yapan ve karakter kaybını
önlemeyi hedefleyen bir haberleşme algoritması. RS485 üzerinden IEC 62056-21
(Kohler AEL.TF.20) elektrik sayacı ile handshake yapıp parametre okumaktadır;
okunan değerler ayrıca bir Xbee modulu üzerinden iletilmek üzere
hazırlanmaktadır (frame oluşturma fonksiyonları dahildir).

Bu kod sadece USER CODE bloklarına yazılmış parçaları içerir. Çalıştırmak
için once STM32CubeMX ile bir proje iskeleti oluşturup (USART1/2/4, DMA,
DE/RE kontrol pini) `main.c` dosyasındaki USER CODE bölgelerini buradaki
içerikle değiştirmeniz gerekir.

## Bilinen sınırlamalar

- `HAL_UART_RxCpltCallback` içinde bir sonraki byte için DMA yeniden
  `while(temp == 0){ HAL_UART_Receive_DMA(...); }` şeklinde ana döngüde
  mesleği ile tekrar tetiklenir; dönüş değeri kontrol edilmez. Bu, standart
  HAL DMA kullanım şeklinden (callback içinde yeniden-arm) farklıdır.
- `sendxBee()` içindeki ikinci veri kopyalama blogu, ilkiyle aynı kaynağı
  (`buff[0..3]`) kullanır; ikinci sensör/ölçüm için tamamlanmamış bir
  yer tutucudur.
