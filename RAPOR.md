# H175 İnsansı Robot – Teknik Durum Raporu (2026-11-25)

Proje klasörü: `/home/yagiz/Documents/H175_V2`

## 1. Kapsam ve durum özeti

| Konu | Durum |
|---|---|
| CAD (Asimov-1 tarzı yerleşim), Jetson + kart sığması | Yapıldı (parametrik cadquery) |
| Motor seçimi (40 gövde motoru + 30 parmak ekseni) | Yapıldı, **katalog değerleri doğrulanmadı** |
| Simscape Multibody modeli (serbest gövde + temas + motor sınırları) | Yapıldı |
| Simscape'te yürüme | Yerinde adım 10 s kararlı, ileri yürüme ~0,5 m sonra devriliyor |
| MuJoCo modeli | Yapıldı |
| MuJoCo'da ayakta durma | 10 s kararlı |
| MuJoCo'da yürüme | **Güncel (bkz. 6.6):** 0,4 m/s'ye kadar düz ayakla yürüyor, 0,6 m/s ve üstü devriliyor |
| Çağrının hız/merdiven/eğim/darbe testleri | Yapılmadı, hedeflerin çok altındayız |
| MuJoCo'da motor yükü doğrulaması | 0,25 ve 0,40 m/s için yapıldı, doyma yok (6.6) |

Resmi çağrı sayfası (rip.sanayi.gov.tr) bot engeli verdi. Şartlar, projedeki `gereksinimler.csv` dosyasından (ikincil kaynak) alındı. Resmi Ek-1 ile karşılaştırma yapılmadı.

## 2. Mekanik tasarım (CAD)

Kaynak: `build.py` + `v3_patch.py` → `H175_V2.urdf`, `.step`, `meshes/`, `geometry.json`, `actuators.json`, `fit_check.csv`, `motor_secimi.csv`.

- 1750 mm boy, 71 link, 70 eklem, toplam kütle 69,0 kg.
- 40 gövde ekseni (≥40 şartında tam sınırda) + el başına 15 parmak ekseni.
- **Bacak (her taraf):** hip_pitch → hip_roll → hip_yaw → knee → ankle_pitch → ankle_roll → toe. Kalça merkezi z=880 mm, diz 465 mm, ayak bileği 80 mm. Uyluk 415 mm, baldır 385 mm.
- **Kol:** scapula, clavicle, shoulder pitch/roll/yaw, elbow, forearm twist, wrist pitch/roll/yaw + parmaklar.
- **Gövde:** bel yaw/roll/pitch, boyun yaw/roll/pitch.
- Eklem sırası ve oranlar Asimov-1'den (menloresearch, CERN-OHL-S) esinlenildi. CAD kopyalanmadı, yalnızca topoloji ve oranlar kullanıldı.
- **Elektronik yerleşimi:** Jetson AGX Orin (110×110×72) kafada. İki batarya paketi (~1,5 kWh) sırt bölgesinde. Güç dağıtım kartı, gerçek zamanlı denetleyici, hub ve IMU gövdede. Lidar dışarıdan monte. Hepsi `fit_check.csv`'de "SIĞAR" çıktı. Bu kontrol kutu-kutu geometrik kontroldür, kablo ve soğutma modellenmedi.
- Kafa ve gövde 5 mm duvarlı içi boş kutu olarak modellendi, elektronik kütleleri noktasal atalet olarak eklendi.
- **Bilinen sadeleştirme:** Asimov'daki paralel RSU ayak bileği yerine iki seri eksen (pitch + roll) kullanıldı.

## 3. Motor seçimi

Kaynak: `v3_patch.py` (katalog + tork gereksinimi + kütle bütçesi iterasyonu).

- Katalog: AK60-6 (9 Nm), AK70-10 (24,8), AK10-9 (53), AK80-64 (120), HT-200 (200), HT-300 (300).
- Her eklem için gerekli tepe tork, gövde ağırlığına oranla belirlendi (ör. hip_pitch 0,20·M·g, diz 0,22·M·g, ayak bileği pitch 0,11·M·g). Emniyet katsayısı 1,25. Seçilen motor bu değerin üstündeki en küçük sınıf.
- Motor kütlesi gövde kütlesine geri beslenip tekrar hesaplandı (iteratif kütle bütçesi).
- Sonuç: 9× AK10-9, 11× AK80-64, 1× HT-300, 2× AK60-6, 11× AK70-10, 6× HT-200 (gövde, boyun ve kol dahil toplam 40 motor; parmak eksenleri ayrı).
- **Uyarılar:**
  - CubeMars değerleri hafızadan/hedeften alındı, veri sayfasıyla doğrulanmadı.
  - HT-200 ve HT-300 özel sipariş sınıfı hedef spesifikasyonlardır, hazır ürün değildir.
  - Armature (yansıyan rotor ataleti) değerleri yaklaşıktır.
  - 4,5 saatlik çalışma süresi hesaplanmadı.

## 4. Simscape Multibody modeli

Kaynak: `H175_V3_multibody_raw.slx` (smimport), `build_h175_walk.m`, `run_h175_walk.m`, `h175_gait.m`, `h175_gravity_ff.m`.

- Dünya ile pelvis arasında 6-DOF serbest eklem, 30×6 m zemin, her ayakta 8 küre (r=6 mm) ile Spatial Contact Force (k=1e6, d=2e3, sürtünme 1,4).
- Her eklemde PD + motor torku sınırı: `K(qref−q) − B·w + tau_ff`, tork tavanı `Tpk·sat(2−|w|/Wnl)` (hıza bağlı sınır).
- İleri besleme: Robotics System Toolbox `inverseDynamics`, ground reaction ZMP'ye uygulanıyor, `g_eff = g − a_base` ile atalet dahil.
- Ayak bileği stratejisi: CoM/DCM geri beslemesi, CoP yetkisi ±8 cm ile sınırlı.
- Çözücü: değişken adımlı ode23t. Sabit adımlı Backward-Euler "dejenere kütle" hatası verdi.
- **Sonuç:** Yerinde adım 10 s kararlı (pelvis z≈0,845, y salınımı ±6 cm). İleri yürüme (0,12 m/s) ~5–6 adım, ~0,5 m sonra öne devriliyor (~8,2 s). Motorlar düşmeden önce tepe torkun %85'inin altında kaldı.
- **Çözülemeyen sebep:** Ayaklar çift destekte birkaç cm kayıyor, ağırlık merkezi plandan 3–5 cm geri kalıyor, ayak bileği CoP yetkisi doyuyor. Basamaklama düzeltmesi, DCM kazanç artışı ve sürtünme artışı sorunu geciktirdi ama çözmedi.
- Simscape'te 10 s benzetim dakikalar sürüyor.

## 5. MuJoCo modeli ve kontrol mimarisi

Kaynak: `mujoco/make_model.py`, `mujoco/ctrl.py`, `mujoco/stand_test.py`, `mujoco/walk_test.py`, `mujoco/view_walk.py`, `dump_gait.m`.

### 5.1 Model (`make_model.py`)
- URDF → `MjSpec`; pelvis'e serbest eklem, düzlem zemin (sürtünme 1,2).
- Tüm URDF geometrileri çarpışmadan çıkarıldı. Yalnızca eklenen ayak kutuları temas yapar (ankle_roll üzerinde 18×10 cm, toe üzerinde 10×10 cm). Gövde ve kol çarpışması yoktur, bu yüzden düşen robot zeminin içinden geçer. Düşme `pelvis z<0,7 m` ile algılanıyor.
- Görsel meshler `discardvisual=0` ile korunuyor.
- Zaman adımı 2 ms, entegratör `implicitfast`.
- Eklem sönümü 0,3, sürtünme 0,1 Nm (motorlu eklemler); motorsuz eklemlerde sönüm 0,5, yay 2,0.
- Armature ayrıca eklenmedi, çünkü URDF atalet değerlerine zaten dahil.
- Kütle 69,02 kg (CAD ile uyumlu).

### 5.2 Motor modeli
Her eklem için MuJoCo konum tipi aktüatör (affine bias):
`τ = Kp(ctrl − q) − Kd·q̇`, `Kp = τ_peak/0,15 rad`, `Kd = 0,1·Kp`, `|τ| ≤ τ_peak` (`actuators.json`).
Bu dosyadaki en önemli bulgu: aynı PD'yi dışarıda hesaplayıp tork olarak vermek 2 ms'de kararsızdı (motorlar titreşip robotu devirdi). Aktüatör PD'si implicitfast entegratörle kararlı.
Hıza bağlı tork sınırı (Simscape'teki `Wnl` modeli) MuJoCo'ya **taşınmadı**, yalnızca sabit tepe tork sınırı var.

### 5.3 Statik ileri besleme (`ctrl.py: StaticFF`)
- Gövde 6 serbestlik derecesinin yerçekimi+bias kuvveti, temasta olan ayakların 4'er köşe noktasındaki 3B kuvvetlerle (min-norm en küçük kareler) dengelenir.
- Eklem ileri besleme torku: `τ_ff = bias_eklem − (Jᵀ f)_eklem`.
- Komut: `ctrl = q_ref + τ_ff/Kp`.
- Yürümede destek ayağı, referans ayak yüksekliğinden (`fz < 3 mm`) seçilir; salınan ayak yük dağıtımına katılmaz.
- Atalet (ivme) terimi **yok**, bu hız sınırının muhtemel sebebi (ölçülmedi).

### 5.4 Yan düzlem geri besleme
Destek ayak(lar)ının ankle_roll eklemine:
`τ_roll += 0,5·M·g·(gy·(y_CoM − y_ZMP_ref) + gyv·vy_taban)·destek`, `gy=−2…−3`, `gyv=−1`.
- Kazanç aralığı dar: −2 ile −3 çalışıyor, −5 devriliyor, +1 ve −1 de devriliyor.
- İleri düzlemde geri besleme gerekmedi (0,12 m/s için). Daha yüksek hızlarda denemek (gx=1) fayda vermedi.

### 5.5 Yürüyüş referansı (`h175_gait.m`)
- Kajita ZMP/LIPM ön bakışlı kontrol → CoM yörüngesi (idare ile Riccati).
- Adım planı: sağ destekli başlar, 8–10 adım, adım uzunluğu `vx·Ts`, çift destek oranı `dsr`, ayak yörüngesi 5. derece polinom, adım yüksekliği 4–6 cm.
- Analitik bacak IK (pitch→roll kalça sırası), ayak düz kalıyor, FK ile doğrulandı (ayak bileği hatası ~0, ayak eğimi ≤0,016 rad).
- `dump_gait.m` referansı CSV'ye yazar; MuJoCo bu CSV'yi eklem konum referansı olarak izler. Üst vücut (kol, bel, boyun) sıfırda sabit.

## 6. Sonuçlar

### 6.1 Ayakta durma (MuJoCo, geri beslemesiz)
- 10 s sabit: pelvis z 0,928 m, x kayması 2 mm.
- Pelvise 0,2 s kuvvet: ileri 100 N tamam / 150 N devriliyor; yan 150 N'a kadar tamam (üstü denenmedi).

### 6.2 Yürüme (MuJoCo)
| Referans | Sonuç |
|---|---|
| 0,12 m/s, Ts 1,0 s, 8 adım | Yürüdü, x=0,789 m (plan 0,785), max |y|=10,7 cm, z 0,886–0,899 m |
| 0,25 m/s, Ts 0,8 | 4,3 s'de devrildi |
| 0,40 m/s, Ts 0,7 | 4,0 s'de devrildi |
| 0,60 m/s, Ts 0,6 | 5,3 s'de devrildi |

Yürüme sonucunu tek bir referans üzerinde ve tek bir gürültüsüz benzetimle aldım. Rastgeleleştirme, itme, pürüzlü zemin test edilmedi.

### 6.3 Motor yükü doğrulaması (MuJoCo, 0,12 m/s, 11 s, `mujoco/motor_load.py`, `motor_yuk_mujoco.csv`)
Ayak bileği roll kazancı gy=−3 ile (düşmeyen tek ayar). gy=−2 (5,3 s) ve −2,5 (6,1 s) bu referansta devrildi, yani kazanç penceresi çok dar.

| Eklem | Motor | Tpk (Nm) | Tepe | Tepe/Tpk | RMS (Nm) | RMS/Anma |
|---|---|---|---|---|---|---|
| ankle_roll (L/R) | AK10-9 | 53 | 53 (doyuyor) | **1,00** | 13–14 | 0,7–0,8 |
| hip_roll (L/R) | AK80-64 | 120 | 89 | 0,74 | 32–33 | 0,7 |
| hip_yaw L | AK10-9 | 53 | 35 | 0,66 | 10,5 | 0,6 |
| knee (L/R) | HT-200 | 200 | 108–111 | 0,54–0,55 | 47–50 | 0,6 |
| ankle_pitch (L/R) | AK80-64 | 120 | 39–45 | 0,33–0,37 | 10 | 0,2 |
| waist_roll | AK80-64 | 120 | 26 | 0,21 | 8 | 0,2 |

- Hiçbir eklemde RMS anma torkunu (Trated) geçmedi. Hiçbir eklem hız sınırının (w_noload) %80'ine ulaşmadı. Toplam mekanik güç tepe 213 W, ortalama 30 W.
- **Bulgu (DÜZELTİLDİ, bkz. 6.5):** ankle_roll (AK10-9, 53 Nm) tepe torkta doyuyor. İlk yorumda motorun büyütülmesi önerilmişti; bu yanlıştı. Ayak yarı genişliği 5 cm olduğundan yere basan ayak yaklaşık M·g·0,05 ≈ 34 Nm'den fazla roll torkunu zemine iletemez, fazlası ayağı kenarına devirir. Doyma, geri beslemenin fiziksel olarak uygulanamayan tork istemesinden geliyor.
- hip_roll %74 ile ikinci en yüklü eklem. Kalan eklemler %55 ve altında.
- **Sınırlar:** Yalnızca yavaş yürüme (0,12 m/s) yüklenmiş durumda. 1,4 m/s hedefinde yük çok daha yüksek olur, bu sonuç ondan bir şey söylemez. Tork 10 ms'de bir örneklendi, kısa tepeler kaçmış olabilir. Hıza bağlı tork sınırı modelde yok, hız zaten düşük olduğu için etkisi bu testte yok. Batarya ömrü hesaplanmadı (ort. 30 W yalnızca mekanik güç).

### 6.4 Hız bozulmasının sebebi için ek deneyler (MuJoCo, her biri tek koşu)
- **Destek ayağını gerçek temas kuvvetinden seçmek** (referans yüksekliği yerine): başarılı 0,12 m/s koşusu bu değişiklikle devrildi (8,8 s), 0,25 m/s koşusunda fark yaratmadı (4,4 s). Yani "ayak yeterli yük almadan destek sayılıyor" hipotezi 0,25 m/s'deki bozulmanın sebebi olarak doğrulanmadı.
- **Adım süresi ile adım uzunluğunu ayırma** (gy=−3 / −2): 

| Referans | Adım uzunluğu | Adım süresi Ts | Sonuç (gy=−3 / gy=−2) |
|---|---|---|---|
| 0,12 m/s | 12 cm | 1,0 s | yürüdü / devrildi 5,3 s |
| 0,25 m/s | 25 cm | 1,0 s | devrildi 7,2 s (x=1,13 m) / 5,2 s |
| 0,12 m/s | 9,6 cm | 0,8 s | devrildi 4,3 s / 6,7 s |
| 0,12 m/s | 7 cm | 0,6 s | devrildi 4,9 s / 5,5 s |
| 0,25 m/s | 20 cm | 0,8 s | devrildi 4,1 s / 5,7 s |

Adım uzunluğunu iki katına çıkarmak (Ts sabit) robotu daha uzun ayakta tuttu. Ts'yi 1,0'dan 0,8 s'ye düşürmek ise hız aynı kalsa bile bozuyor. Bu, sınırlayıcının ileri düzlemden çok adım süresine bağlı olduğunu düşündürüyor (örneğin yan düzlemde ağırlık aktarımı hızı ve roll kazanç penceresi). Henüz her koşuda ilk bozulan sinyali (ayak kayması, temas kuvveti, CoM hatası) zaman ekseninde incelemedim. Bu bir sonraki adım.

### 6.5 İlk bozulan sinyal (`mujoco/diag_walk.py`, `diag_walk.png`; her referans tek koşu, gy=−3)
- **Başarılı 0,12 m/s yürüyüşü de temiz değil:** destek sayılan ayak 22,9° roll yapıyor (pitch en çok 2,4°), yani ayak kenarında yuvarlanıyor. Destek başına 6 cm'ye varan kayma var. Bu yürüyüş sınırda bir sonuçtur, "kararlı yürüme" olarak sunulmamalı.
- **Ts 0,8 s koşuları (0,12 ve 0,25 m/s aynı desen):** ilk tek destekte (sağ ayak) pelvis plandan destek tarafına doğru fazla gidiyor (y hatası t≈2,9 s'de eşiği aşıyor), sağ ayak roll yapıyor, taban eğimi ve ardından ileri hata büyüyor. İlk büyüyen sapma yan düzlemde; ileri hata sonra geliyor.
- **0,25 m/s Ts 1,0 s:** önce ileri hata eşiği aşıyor (3,75 s), robot birkaç adım dayanıyor, 6,6 s'de yan düzlemde kopuyor.
- Eşikler başarılı koşunun tepe değerlerinin 1,3 katıdır. "Planlı destek yüksüz" olayı başarılı koşuda da var, ayırt edici değil.
- **Roll geri besleme torkunu fiziksel sınıra kırpma denemesi** (±0,045·M·g ve ±0,03·M·g Nm): doyma büyük ölçüde kayboluyor ama robot daha iyi yürümüyor; başarılı koşu bile devriliyor (8,7 ve 7,9 s). Yani yürüyüş şu an ayak kenarına yük bindiren aşırı torka dayanıyor ve yan düzlem kontrol stratejisinin değişmesi gerekiyor.
- Doğrulanmamış aday sebep: `StaticFF` çift destekte yükü plandaki ZMP'ye göre değil min-norm paylaştırıyor ve atalet terimi yok.

### 6.6 Yeni kontrolcü `mujoco/wbc.py` (güncel durum; 6.2–6.5'teki yürüme sonuçları ve 5.3–5.4 kontrolü eski kontrolcüye aittir)
**Yöntem:** (1) referans konum/hız/ivmeden ters dinamik `M·q̈+h` (ataletsel ileri besleme); (2) temas kuvvetleri ayak köşelerinde, sürtünme piramidi içinde ve ayak alanı dışına çıkamayacak şekilde NNLS ile çözülür, çift destekte yük plandaki ZMP'ye göre paylaşılır; (3) denge: DCM hatasıyla orantılı ZMP kayması gövde kuvvetine eklenir (ayak bileğine doğrudan tork yok); (4) destek kalçalarında gövde dikliği geri beslemesi; bacak PD'si 2× sertleştirildi (1× ile de yürüyor).

| Referans | Sonuç (x / plan) | Max destek ayağı eğimi | Kayma |
|---|---|---|---|
| 0,12 m/s, 8 adım | 0,80 / 0,79 m | 0,9° | 0,2 cm |
| 0,25 m/s | 1,71 / 1,71 m | 0,5° | 0,2 cm |
| 0,25 m/s, 30 adım | 5,65 / 5,71 m | 5,3° | 1,0 cm |
| 0,40 m/s | 2,94 / 2,95 m | 1,2° | 0,5 cm |
| 0,60 / 0,8 / 1,0 / 1,4 m/s | devriliyor (7,9 / 6,1 / 5,2 / 4,1 s) | | |

- Salınan ayak havada kalma oranı %96–100 (0,4 m/s'ye kadar), yani sürünme değil.
- Uzun koşuda (30 adım) eğim 3° hedefinin üstünde; adım yeri yeniden planlanmıyor.
- **Gürbüzlük (0,25 m/s, kontrolcü bozulmayı bilmiyor):** sürtünme 0,1–0,6, gövde +7 kg, gövde CoM +8 cm, 0,2 s'lik 100 N ileri/geri ve 70 N yan itme tamam. 100 N yan itme ve +25 kg devirir. kdcm=3 ile 100 N yan ve 200 N ileri itme de geçiyor (200 N geri devirir) ama 30 adımlık koşu devriliyor; varsayılan 1,5 bırakıldı.
- **Motor yükü (0,25 ve 0,40 m/s):** en yüklü hip_yaw %70, hip_roll %63, diz %49, ankle_roll en çok %53. Doyma yok, RMS anma torkunun altında, hız w_noload'un %50'sinin altında. Mekanik güç ort. 38 W (0,25), 74 W (0,40).
- **Sınırlar:** durum (pelvis konum/yön/hız) kusursuz biliniyor, sensör gürültüsü ve gecikme yok; zemin düz ve sert; MuJoCo modelinde tork-hız eğrisi yok; gövde/kol çarpışması yok; Simscape'e taşınmadı; adım yeri geri beslemesiz.

## 7. Bilinen sınırlar ve dürüst notlar

- Çağrıdaki hız hedefleri (1,4–1,6 m/s), merdiven, eğim ve darbe testleri karşılanmadı. Mevcut yürüyüş 0,12 m/s.
- Motor değerleri (katalog) doğrulanmadı. MuJoCo yürüyüş yükü yalnızca 0,12 m/s için ölçüldü (bkz. 6.3 ve 6.5).
- Simscape'teki hıza bağlı tork sınırı, ayak bileği CoP sınırı ve ayak şekli MuJoCo'da aynı değil. İki modelin sonuçları doğrudan karşılaştırılamaz. MuJoCo'nun yürümesinin nedenini (yumuşak temas modeli, farklı ayak geometrisi, farklı PD) ayrıştırmadım.
- Yan geri besleme kazancı hassas. Gürbüzlük kanıtlanmadı.
- Parmaklar ve kollar yürüyüşte pasif (PD ile sıfırda).
- RSU paralel ayak bileği, kablo/soğutma, gerçek motor sürücü gecikmeleri modellenmedi.
- Asimov'un yürüyüş politikası yayımlanmadığı için doğrudan karşılaştırma yapılamadı; yalnızca topoloji referans alındı.

## 8. Dosya haritası

| Dosya | Görev |
|---|---|
| `build.py`, `v3_patch.py` | CAD, motor seçimi, URDF/STEP üretimi |
| `actuators.json`, `motor_secimi.csv`, `fit_check.csv` | Motor ve sığma çıktıları |
| `H175_V2.urdf`, `meshes/` | Robot tanımı |
| `h175_gait.m`, `h175_gravity_ff.m` | Yürüyüş referansı, ileri besleme (Simscape) |
| `build_h175_walk.m`, `run_h175_walk.m`, `H175_V3_walk.slx` | Simscape yürüme modeli |
| `h175_kinematics.m`, `build_h175_control.m` | Eski V2 isimleriyle yazıldı, V3 eklem adlarına güncellenmedi |
| `dump_gait.m` | Referansı MuJoCo için CSV'ye yazar |
| `mujoco/make_model.py`, `ctrl.py` | MuJoCo modeli ve ileri besleme |
| `mujoco/stand_test.py`, `walk_test.py` | Duruş ve yürüme testleri |
| `mujoco/view_walk.py` | Yürüyüşü pencerede izler |
| `mujoco/gait_ref.csv`, `g25/g40/g60.csv` | Yürüyüş referansları |

## 9. Önerilen sıra

1. **Yan düzlem kontrolünü düzelt:** destek ayağı devrilmeden (roll torku ≤ ~34 Nm) ağırlık merkezini kalça/gövde veya adım yeri üzerinden düzelt. ankle_roll motorunu büyütme gerekçesi yok.
2. **Ataletsel ileri besleme:** `mj_inverse` ile ivme terimini ekleyip 0,25–0,5 m/s referanslarını yeniden dene.
3. **RL (ZMP referanslı):** RTX 3060 Laptop GPU mevcut. Saatler sürer, kararlılık garantisi yok.
4. **Motor veri sayfaları:** CubeMars değerlerini doğrulayıp özel sınıf motorlar için üretici ara.
5. **Resmi şartname:** Ek-1'i indirip `gereksinimler.csv` ile karşılaştır.
