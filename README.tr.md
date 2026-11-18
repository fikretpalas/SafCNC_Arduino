# SafCNC Arduino Yazılımı

**Arduino Uno / Nano** (ATmega328P, 16 MHz) kartını **Mach3** için 3 eksenli USB hareket
denetleyicisine çeviren kart yazılımı. PC hareketi 5 ms'lik dilimler halinde gönderir; kart bunların bir
saniyelik kısmını tamponda tutar ve adım / yön sinyallerini 32 kHz zamanlayıcı kesmesinde üretir.

**[English](README.md)**

| | |
|---|---|
| Mach3 eklentisi (Türkçe kaynak) | [SafCNC_Mach3Plugin_tr](https://github.com/fikretpalas/SafCNC_Mach3Plugin_tr) |
| Mach3 eklentisi (İngilizce kaynak) | [SafCNC_Mach3Plugin_en](https://github.com/fikretpalas/SafCNC_Mach3Plugin_en) |

Kaynak kod, dosya adları ve ayar adları İngilizcedir; bu sayfa yalnızca Türkçe açıklamadır.

## Özellikler

* 3 eksen; 5 ms'lik dilimde 160 adıma kadar = eksen başına 32 kHz; adımlar DDA ile eşit yayılır
* Üç eksenin darbeleri doğrudan port erişimiyle birlikte verilir
* 200 dilimlik halka tampon (1 saniyelik hareket)
* GRBL 1.1 ve Arduino CNC Shield V3.51 ile uyumlu pin düzeni
* Limit ve prob girişleri süzülür ve adım kesmesinin içinde denetlenir
* PC durdurmasa bile makineyi durduran yerel acil stop girişi
* 200 ms bağlantı bekçisi; her pakette sıra sayacı ve 16 bit sağlama toplamı
* Duruştan sonra kart tam dilimi ve alt vuruşu bildirir; PC gerçek konumu geri kurabilir
* Hız sınırını (`STEP_LIMIT`, dilim başına 160 adım = 32 kHz) açılışta ve her yeni bağlantıda eklentiye
  bildirir; eklenti hareketi bu hıza göre planlar
* Fener mili PWM pini, fener mili yönü, soğutma ve sis çıkışları
* Tüm ayarlar derleme anı sabitidir ve `static_assert` ile denetlenir

## Pin düzeni

| Pin | Görev | Pin | Görev | Pin | Görev |
|---|---|---|---|---|---|
| D2 | X adım | D5 | X yön | D9 | X limit |
| D3 | Y adım | D6 | Y yön | D10 | Y limit |
| D4 | Z adım | D7 | Z yön | D12 | Z limit |
| D8 | Step sürücü etkinleştirme (etkin DÜŞÜK) | D11 | Fener mili PWM | D13 | Fener mili yönü |
| A0 | Acil stop | A1 | Feed Hold (duraklat) | A2 | Cycle Start (başlat) |
| A3 | Soğutma sıvısı | A4 | Sis soğutma | A5 | Prob |

Girişler etkin DÜŞÜK'tür ve dahili pull-up dirençleri açıktır: her anahtarı pin ile GND arasına bağlayın.
CNC Shield V3.00 kopyalarında Z limit *SpnEn*, fener mili PWM ise *Z+* bağlantısındadır.

## Kurulum

1. Bu depoyu ZIP olarak indirin (*Code > Download ZIP*) ve Arduino IDE'de
   *Taslak > Kütüphane Ekle > .ZIP Kütüphanesi Ekle* ile kurun.
2. *Dosya > Örnekler > SafCNC > SafCNC_Basic* örneğini açın.
3. *Arduino Uno* ya da *Arduino Nano* seçip yükleyin.
4. Yukarıdaki depolardan birindeki Mach3 eklentisini kurun.

Taslağın tamamı şudur:

```cpp
#include <SafCNC.h>

void setup() { SafCNC.begin(); }
void loop()  { SafCNC.update(); }
```

Taslakta `delay()` ya da `Serial` kullanmayın: seri port kart yazılımına aittir ve `update()`
olabildiğince sık çalışmalıdır.

## Ayarlar

Kitaplık klasöründeki `src/SafCNC_Config.h` dosyasını düzenleyip yeniden yükleyin.

| Ayar | Varsayılan | Anlamı |
|---|---|---|
| `STEP_INVERT`, `DIR_INVERT` | `0x00` | Eksen bazında adım / yön tersleme (bit 0 = X, 1 = Y, 2 = Z) |
| `ENABLE_ACTIVE_LOW` | `1` | Step sürücü etkinleştirme polaritesi (A4988, DRV8825, CNC Shield) |
| `INPUT_INVERT` | `0x00` | Girişleri tek tek tersleme |
| `LOCAL_ESTOP` | `1` | A0'daki acil stop girişi kartı doğrudan durdurur |
| `ESTOP_DEBOUNCE_MS` | `5` | Acil stop süzme (debounce) süresi |
| `HOLD_TORQUE_ON_FAULT` | `1` | Acil stop dışındaki hatalardan sonra sürücüler açık kalır |
| `LIMIT_FILTER`, `PROBE_FILTER` | `3`, `2` | Girişin art arda etkin olması gereken adım vuruşu sayısı |
| `SPINDLE_PWM_MODE` | `1` | Fener mili PWM frekansı: ~490 Hz, ~980 Hz ya da ~7,8 kHz |

Dosyanın sonundaki zamanlama sabitleri protokolü tanımlar; yalnızca eklentiyle birlikte değiştirin.

## Bağlanma süresi

Arduino önyükleyicisi (bootloader) varken kartın bağlanması yaklaşık 7 saniye sürer; çünkü port her
açıldığında önce önyükleyici çalışır. ISP programlayıcıyla yüklerseniz (*Taslak > Programlayıcı
Kullanarak Yükle*) önyükleyici silinir ve kart neredeyse anında bağlanır. *Araçlar > Önyükleyiciyi
Yazdır* normal USB yüklemeyi geri getirir.

## Kısaca protokol

| Yön | Paket | Boy | İçerik |
|---|---|---|---|
| PC -> kart | durum (`K_RESET`, `K_READY`, `K_ACTIVE`) | 8 bayt | komut, sayaç, sıfırlama nedeni ya da fener mili + çıkışlar, sağlama |
| PC -> kart | hareket (`K_MOVE`) | 10 bayt | komut, sayaç, sonraki 5 ms için işaretli X / Y / Z adımları, sağlama |
| kart -> PC | durum, 50 ms'de bir | 8 bayt | durum, sayaç, tampon doluluğu, fener mili, çıkışlar, girişler, sağlama |
| kart -> PC | bilgi (`K_INFO`), açılışta ve yeni bağlantıda | 8 bayt | komut, sayaç, `STEP_LIMIT` (16 bit), `SUBTICKS` (16 bit), sağlama |

Kart durumları: `RESET` (durmuş, PC bekleniyor), `READY` (bağlantı kurulu, hareket yok), `ACTIVE`
(hareket kabul eder), `MOVE` (yürütüyor). Her protokol hatası, zaman aşımı, limit, prob ya da acil
stop olayı kartı `RESET` durumuna döndürür ve neden kodu PC'ye gönderilir.

Protokol bu karta özgü değildir. Mach3 eklentisi aynı protokolü uygulayan başka kartları da
sürebilir; STM32 kart yaklaşık 100 kHz'e çıkabilir (test edilmedi); Arduino ise yaklaşık 30 kHz ile sınırlıdır.
Her kart kendi hızını `K_INFO` paketiyle bildirir (adım hızı = `STEP_LIMIT` x 200 Hz); protokol 5 ms'lik
dilimde en fazla 500 adıma (100 kHz) izin verir.

> CNC makineleri insanlara zarar verebilir ve takımları kırabilir. Her zaman motor ve fener mili gücünü
> kesen donanımsal bir acil stop kullanın. Bu yazılım hiçbir garanti verilmeden sunulmaktadır.

## Geliştirici

**Fikret Palas** - <safcnc@tutamail.com> - <https://github.com/fikretpalas>

## Lisans

[MIT](LICENSE)
