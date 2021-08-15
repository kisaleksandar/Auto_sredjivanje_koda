# PARKING SENZOR- ZADATAK PROJEKTA OKVIRNO
-Potrebno je napraviti RTOS koji ce simulirati sistem za kontrolu parkinga pomocu dva senzora

-Simulirati vrednosti senzora pomocu serijske komunikacije AdvUniCom

-Kalibrisati obe vrednosti senzora

-Na osnovu kalibrisanih vrednosti genesirati signale:
    Ako je kalibracija izmedju 50% i 100%: neka blinkaju diode frekvencijom 1Hz - takodje ispisvati na terminalu _DETEKCIJA
    Ako je kalibracija izmedju 0% i 50%: neka blinkaciju diode frekvencijom 2Hz - BLISKA DETEKCIJA
    Ako je kalibracija manja od 0: neka diode svetle periodom 0.5s - KONTAKT DETEKCIJA
    OVO ODRADITI ZA OBA SENZORA ZASEBNO
    
-Napraviti start/stop sistem, gde se sistem moze paliti prekidacem, gasiti porukom STOP, ili paliti porukom START, gasiti prekidacem itd.

-Sve dok je sistem aktivan neka bude upaljena jedna dioda, i neka se ugasi kada se sistem stopira(ugasi)

-Na terminalu serijske ispisivati trenutno stanje sistema, da li je startovan ili stopiram, i ako je starovan ispisivati kalibrisane vrednost sa senzora

-MisraC

# PERIFERIJE

-Prilikom pokretanja LED_bars_plus u komandnu liniju ukucati LED_bars_plus.exe rRRR, jer je potrebno imati jedan stubac(prvi sa leva) kao ulazni
 ,a ostala tri stubca izlazni.
 
 -Prilikom pokretanja serijske komunikacije, potrebno je aktivirati tri zasebna kanala AdvUniCom. U komandnu liniju kucati redom komande
    AdvUniCom
    AdvUniCom 1
    AdvUniCom 2

-Prilikom pokretanja sedmosegmentnog displeja kucati komandu Seg7_Mux 9

# KAKO TESTIRATI SISTEM
-Kada su pokrenute sve periferije na nacin opisan iznad, moze se pokrenuti kod i testirati sistem.

-Sistem se moze aktivirati preko kanala2 AdvUniCom, slanjem komande START/0d. Komanda se kuca u box koji se nalazi levo od komande SEND CODE. Kada je ova komanda
poslata, na led baru treba da se upali skroz donja dioda drugog stubca LED_bar-a. Na Terminalu kanala 2, stanje sistema ce da se promeni iz START u STOP(Stanje:START...) i
krenuce da se prikazuju kalibrisane vrednost oba senzora(npr. K1:072, K2:050). 

-Ukoliko zelimo slati vrednosti sa senzora, to cinimo preko kanala 0(senzor1 iliti levi_senzor) i preko kanala 1(senzor2 iliti desni_senzor). Takodje vrednost kucamo u box levo
od komande SEND CODE, i kucamo je na nacin: 80/0d (ovde smo poslali npr 80cm sa senzora). 

-Kada posaljemo vrednosti sa oba senzora preko AdvUniCom kanala 0 i 1. Na kanalu 2 bi trebale da nam se prikazuju kalibrisane vrednosti(npr. ako smo poslali 60cm sa jednog kanala,
kalibrisana vrednost je 50%(jer je za minimum uzeto 20cm, a za maksimum 100cm)).

-U zavisnosti od kalibrisanih vrednosti senzora 1, gornje cetiri diode treceg stubca sa leva ce blinkati(Zavisi u kojoj zoni se nalazi objekat koji je detektovan senzorom).
Ista stvaar vazi i za kalibrisane vrednosti sa senzora 2, samo ce sada blinkati diode CETVRTOG stubca sa leva.

-Sistem mozemo stopirati komandom STOP koja se salje na isti nacin, i kuca na isto mesto kao i komanda START. Ali se moze i ugasiti preko prekidaca, tako sto cemo ga prvo upaliti,
da se poklapa sa komandom START, a zatim ugasiti i tako stopirati sistem.

-Na sedmosegmentnom displeju se ispisuju redom: prvi cifra ako je 0(sistem ugasen), ako je 1(sistem upaljen)
                                                naredne tri cifre predstavljaju vrednost sa senzora 1( bas pravu vrednost u centimentrima)
                                                naredne tri cifre predstavljaju vrednost sa senzora 2( bas pravu vrednost u centimentrima)
                                                
-Na terminalu koji je otvori priliko pokretanja koda(veliki crni terminal), ispisuju se zone detekcije, takodje i da li smo dobro uneli komande START i STOP, ako su dobro unete
na terminalu ce se ispisati Dobro uneseno START ili Dobro uneseno STOP.

# OPIS TASKOVA - led_bar_tsk

-Ovaj task nam sluzi iskljucivo za proveru da li je pritisnut skroz donji taster prvog stubca. Ukoliko jeste palimo skroz donju diodu drugog stubca kao indikaciju
da je sistem upaljen, postavljano promeljivu start na 1 i saljemo to u queue u koji se skladiste start/stop komande. Ukoliko nije pritisnut, ili ga je korisnik iskljucio(prebacio u drugo stanje), gasimo
prethodno upaljenu diodu, start postavljamo na 0 i to saljemo u isti queue.

# OPIS TASKOVA - LED_bar_Task1 i LED_bar_Task2

-Ovde ce biti opisana dva taska, koju su skoro indenticna ali se ipak razliku u jednoj bitnoj stvari. 

-LED_bar_Task1 sluzi iskljucivo za generisanje signala odnosno za blinkanje gornje cetiri diode TRECEG stubca LED_bar-a, u zavisnosti od kalibrisanih vrednosti senzora 1!

-Prvo risivujemo vrednosti sa senzora 1 iz queue-a, zatim pomocu if-ova proveravamo u kojoj se zoni detekcije nalazi objekat detekvovan senzorom 1 i na osnovu toga
na terminal(onaj veliki crni terminal, ne na terminal od serijske) saljemo LEVI_SENZOR:ZONA_DETEKCIJE(npr.), i generisemo da nam diode blinkaju odredjenom frekvecijom.

-Za istu primenu se koristi i LED_bar_Task2, ali on risivuje i obradjuje kalibrisane vrednosti sa senzora 2 i na osnovu tih vrednosti salje odgovarajucu poruku na terminal i
generise blinkanje gornje cetiri diode CETVRTOG stubca LED_bar-a. 

ZASTO OVO U DVA POSEBNA TASKA? - Kad bi ovo bilo smesteno u isti task, while(1) bi prekidali na dva mesta, prvo da proveri u kojoj zoni detekcije se nalazi objekat detektovan
senzorom1, zatim bi izgenerisao signal i poruku. Sve to vreme bi vrednost sa senzora dva morala da ceka da se to izvrsi, pa tek zatim izvrsi istu radnju. Ovime se narusava RTOS
koncept, jer je poenta da se ove dve stvari odvijaju paralelno. Tako ce biti i merodavnije generisanje signala(blikanje dioda) jer nece morati jedan signal da ceka da drugi bude 
izgenerisan da bi se generisao, vec pomocu dva taska paralelno poredimo obe kalibrisane vrednosti i na osnovu tih vrednosti paralelno generisamo odgovarajuce signale.

# OPIS TASKOVA - Primio_kanal_0 i Primio_kanal_1
-Ovde ce biti opisana dva taska, koju su skoro identicna.

-Ovaj task nam sluzi da primamo vrednosti sa senzora tj. sa nultog kanala serijske, da ih konvertujemo pomocu funkcije atof u type double jer nam to odgovara.
Sve sto se primi sa kanala se smesta u promeljivu cc. Zatim se iz te promeljive smesta karakter po karakter u niz rastojanje_kanal0 sve dok ne stigne karakter koji oznacava 
kraj poruke(0x0d). Kada stigne 0x0d, sve sto je smesteno u niz rastojanje_kanal0 se pomocu funkcije atof() konvertuje u type double i smesta u promenljivu senzor1. Zatim vred
nost smestenu u promenljivu senzo1 preko xQueueSend smestamo u queue_senzor1 kako bi te vrednosti mogli da koristimo(risivujemo) u drugim taskovima. Odmah nakon toga u 
promenljivu kalibracija1 smestamo kalibrisanu vrednost prema formuli kalibracija1=100 * (senzor1 - min)/(max-min); Zatim te vrednosti saljemo preko xQueueSend u odgovarajuce
queue-eve da mozemo da ih risivujemo gde nam to treba.

-Task Primio_kanal_1 radi istu stvari, samo za desni senzor, iliti senzor2!!!

# OPIS TASKOVA - SerialReceive_Task

-Ovaj task nam sluzi da obradjujemo poruke primljene sa kanala 2 serijske. Tu korisnik salje komande za START i STOP sistema.

-Klasicno, prvo se komande smestaju u promeljivu cc, u r_buffer niz smestamo karakter po karakter dok ne dodje karakter da je kraj poruke(0x0d).

-Zatim u nastavku taska, poredimo rec koju je stigla sa recju START. Ukoliko je korisnik uneo START, onda setuje promenljivu start=1, saljemo na terminal(veci crni terminal) 
DOBRO UNESENO START i setujemo skroz donju diodu drugog stubca na keca, kao indikaciju da je sistem upaljen.

-Proveravamo takodje da li korsnik mozda uneo STOP, ako jeste, ista procedura kao za START samo gasimo sistem.

# OPIS TASKOVA - Seg7_ispis_task

-Ovaj task nam sluzi za ispis na seg7 displeju. Na njemu ispisujemo vrednosti sa senzora(prave, ne kalibrisane) i start/stop komande.

Prvo risivujemo vrednosti sa oba senzora, zatim i start/stop komande.

Proveravamo da li je start na 1, tj. da li je sistem aktivan. Ako jeste na prvu cifru displeja sa leva ispisujemo 1, u suprotnom nulu.
                                                              Sledece tri cifre ispisujemo vrednosti sa senzora 1.
                                                              Sledece tri cifre ispisujemo vrednosti sa senzora 2.
# OPIS TASKOVA - Serijska_stanje_task

-Ovaj task nam sluzi da ispisujemo trenutno stanje sistema, i trenutno stanje kalibracija na terminalu kanala 2 serijske komunikacije.

-Risivujemo vrednosti kalibracija, zatim i start/stop. Zatim se vrsi ispis.

-Takodje ovaj task tejkuje semafor serijska_stanje svakih 5s. To je namesteno preko tajmer_callback funkcije koja givuje taj semafor ovom tasku na svakih 5s.





























     
