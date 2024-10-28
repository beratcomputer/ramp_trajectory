# ramp_trajectory

# ramp_trajectory
Bu kod herhangi bir hareket sisteminde rampa hareket plani olusturmak icin hazirlanmistir.
- BeratComputer

rampa trajecotrysindeki degiskenlerin bir birimi yoktur. 
x(t) , v(t), a(t) olarak ayrilan pzoisyon ve 2 turevi bulunmaktadir. tum degerler birimden bagimsiz oldugundan en bastaki konumun birimi turevlerinin de birimlerini belirlemektedir.
zaman birimi saniyedir.

ornegin: 3 saniyede 100 metre ilerlemesini hesaplatmak istiyorsaniz artik x degerinizin birimi metre, v (hiz) biriminizin
degeri m/s , a(ivme) degerinizin birimi m/s^2 olacaktir.
### 1.0

bu kodun iki ana fonksiyonu vardir. birisi createTrajectory, digeri ise goWithTrajectory. bu fonksiyonlarin ilki gerekli rampa trajectory sini olusturmak icin verilen girdileri belli kontrollerden gecirerek trajectory olusturur.
olusturdugu trajectorynin tum karakterini anlatabilecek degerlerin ciktisini verir. 

Bir rampa trajectorysini tanimlayan degiskenler:
### 2.0
### 3.0



goWithTrajectory fonksiyonu ise olusturlan trajectorynin ozelliklerine gore surekli olarak fonksiyona girdigi anda hangi pozisyonda olmasi gerektigini dondurur. bu ikinci fonksiyonu surekli cagirarak verdigi degeri setpoint olarak alinmasi gerekiyor.
(bu fonksiyon surekli olarak pozisyon vermesinin yani sira surekli olarak hiz da vermesi yararli olabilir.)

iki fonksiyonun birlikte olan kullanim sekli blok semasi olarak verilmistir.
### 4.0





# CreateTrajectory fonksiyonunun ic yapisi:

oncelikle girdilerden sadece bazilari ile bile ise yarar bir trajectoryi olusturmasi isteniyor.
girdilerden Vmax in girilmemesi durumunda sistemin bir hiz limiti kesinlikle olmali. pratikte her turlu hareket sisteminde bir hiz siniri olacagindan bu limit tanimlanmali. 

- bu durumda Vmax kullanici kullanirken verilmese de sistemin halihazirda bir Vmax i olacagindan Vmax in girilmedigi durum diye bir kontrole ihtiyacimiz yok.
- bir hareket olusturmak icin hedef pozisyonunun da girilmesi zorunlu oldugundan sadece iki parametrenin girilme ve girilmeme olasiligi olmus oluyor: acc , time

bu iki parametrelerin verilmis ya da verilmemis olmalarindaki kombinasyon sayisi 4 oldugundan teker teker inceleyelim.

## ikisi de veririldi ise:
    burada verilen sureye uyabilmek icin trajectoryde ayarlayabildigimiz tek yer sabit hiz ile ilerledigimiz sure.
    x = vp.t2 + a.t1^2
    - buradan t1 ve t2 degerlerini tek bir degiskene dusurulur. t2 = T-2.t1
    - vp = a.t1
    x =  a.t1.(T-2.t1) + a.t1^2
    burada bilinmeyen tek degisken t1 e indirgenince diskriminant ile olasi t1 degerleri cozulur.
    diskriminant cozumu negatif cikar ise belirlenen surede gitmek mumkun olmadigi cikacaktir.

    

## sadece zaman verildi ise:
    sabit hiz ile ilerleme suresini mumkun oldugunca azaltmak amaclanmaktadir. bundan dolayi acc degerinin mumkun oldugu kadariyla az belirlemek gerekmektedir.

## sadece acc verildi ise:
    burada bir zaman ihtiyaci olmadigindan tek uygulanabilir

## ikisi de verilmedi ise:
    burada kullanicinin pek bir kriteri olmadigindan Vmax degerine 2 saniyede ulasabilecegi bir acc secilir. 



# BURALARI TEKRAR DUZENLE!!!!!!!!!!

###### icersinden bahsedelim tekrardan

# CreateTrajectory fonksiyonun icerigi.


oncelikle bir adet dskriminant formulu cozucu kisimi var bu fonksiyonun. bu diskriminant tam olarak t1 koklerini bulmak icin kullaniliyor. Her durumda kullanilamiyor. bu dskriminant formulunun nereden geldigi asagida verilmistir.

## 5.0    T^2 - 4(x/a)

bu cikan diskriminant konuma (x), zamana(T) ve ivmeye(a) baglidir. buradan gerekli t1 degerini bularak trajectory cizilebilir. buradan cikan trajectoryde maximum hiz kontrolu koyulmasi gerekmektedir cunku cikan trajectoryde maksimum hiz geciliyor ise bu trajectorynin uygulanmasi mumkun degildir.

eger diskriminant 0 dan kucuk cikar ise verilen zaman ve verilen ivme degeriyle gitmek mumkun degildir.
bu diskriminanta girdi olarak verilecek senaryolar hangileri?



## FLOW CHART of CreateTrajectory
buraya flow charti koy.


