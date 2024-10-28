import numpy as np
import matplotlib.pyplot as plt

# Girdi parametreleri
def rampa_grafigi(t1, t2, vp):
    # Toplam süre
    toplam_sure = 2 * t1 + t2
    
    # Zaman ekseni
    zaman = np.linspace(0, toplam_sure, 1000)
    
    # Hız grafiği için boş dizi
    hiz = np.zeros_like(zaman)
    
    # Bölümler: Hızlanma (0'dan vp'ye), sabit hız, yavaşlama (vp'den 0'a)
    
    # 1. Hızlanma bölgesi (0 ile t1 arasında)
    ivme = vp / t1
    hiz[zaman <= t1] = ivme * zaman[zaman <= t1]
    
    # 2. Sabit hız bölgesi (t1 ile t1 + t2 arasında)
    hiz[(zaman > t1) & (zaman <= t1 + t2)] = vp
    
    # 3. Yavaşlama bölgesi (t1 + t2 ile toplam süre arasında)
    yavaslama_baslangici = t1 + t2
    hiz[zaman > yavaslama_baslangici] = vp - ivme * (zaman[zaman > yavaslama_baslangici] - yavaslama_baslangici)
    
    # Grafiği çiz
    plt.plot(zaman, hiz, label=f't1: {t1}s, t2: {t2}s, vp: {vp} m/s')
    plt.xlabel('Zaman (s)')
    plt.ylabel('Hız (m/s)')
    plt.title('Rampa Hız Grafiği')
    plt.grid(True)
    plt.legend()
    plt.show()

# Girdi değerleri
t1 = 0.31 # örneğin 2 saniye
t2 = 0  # örneğin 3 saniye
vp = 63.24  # örneğin 5 m/s

# Rampa grafiğini çiz
print(f"position = {vp*t1 + vp*t2}")
print(f"acc = {vp/t1}")

rampa_grafigi(t1, t2, vp)

