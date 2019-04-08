

	;; assemble with nasm & ld:
	;; nasm -f elf hexdumper.asm 
	;; ld -m elf_i386 -o hexdumper hexdumper.o

	;; usage: ./hexdumper filename -a
	;; -a switch is optional and displays corresponding ASCII (only traditional 128 charset)
	
	%define	O_RDONLY	00 ;syˆtet‰‰n flags 0:na, mutta koodi on selke‰mp‰‰, kun siin‰ k‰ytt‰‰ tuota flagsin kirjainmuotoa, joka siis vastaa nollaa
	
segment .data
	valil	db 32		 ;valilyˆnnin ASCII-arvo (dec.)
	rivinv	db 10		 ;rivinvaihdon ASCII-arvo (dec.)
	pviiva	db 124		 ;pystyviivan ASCII-arvo
	piste	db 46		 ;pisteen ASCII-arvo
	ei_loyd db 'Virhe: Tiedostoa ei lˆydy', 0
	len2	equ $ - ei_loyd
	ei_args db 'Virhe: Et antanut tiedoston nime‰', 0
	len     equ $ - ei_args

segment .bss
	;; Oleellista, ett‰ n‰iss‰ muistipaikoissa ei ole sis‰llˆn koko vaan koko tulee sen mukaan, millaisilla rekistereill‰ niit‰ k‰ytet‰‰n
	loppu	resb 1		;boolelainen muuttuja, joka kertoo, ollaanko luettavan tiedoston lopussa
	ascii	resb 1		;0 = asciit pois, 1 = asciit p‰‰ll‰, 2 = ollaan tekem‰ss‰ viimeist‰ ascii-rivi‰
	vkount	resb 1		;kountteri v‰lilyˆnneille, jotka tulee joka nelj‰nnelle tavulle
	rkount	resb 1		;kountteri riveille
	argm	resb 1		;argumenttien m‰‰r‰

	fname	resd 1		;pointteri tiedostonnimeen osoittavaan argumenttiin
	buffer	resd 1		;tavun kokoinen bufferi, luetaan tiedostoa tavu kerrallaan
	fd	resd 1		;file descriptorin paikka, koko on double, koska vaikka FD itse on vain byten, muistipaikkaa operoidaan EAX:ill‰ (32-bit = double)
	ekaa	resd 1		;ensimm‰inen printattava heksa
	tokaa	resd 1		;toinen printattava heksa

	statbuf	resq 11		;88 tavua (11 quad wordia) on struct statin koko
	
segment .text
	global _start
_start:	
	;; ESP talteen ja EBP osoittamaan komentoriviargumentteja, ebp + 4 = argumentit
	push 	ebp		
	mov	ebp, esp
	cmp	dword [ebp + 4], 1
	jne	arg_loytyi
	;; jos ei argumentteja:
	mov     edx, len-1	;viestin pituus
	mov     ecx, ei_args	;itse viesti (pointteri)
	mov     ebx, 1          ;file descriptor (stdout)
	mov     eax, 4          ;system call number (sys_write)
	int     0x80            ;int = software interrupt
	jmp	lopetus

arg_loytyi:
	mov	eax, dword [ebp + 4]
	dec	eax 		;t‰h‰n tulee aina yht‰ suurempi luku kuin mik‰ on argumentti, joten DEC
	mov	[argm], eax
	mov	edx, 3		;t‰st‰ kountteri k‰sitellyille argumenteille, v‰hennet‰‰n aina kaksi, niin on kountteriarvo, muuten l‰hdet‰‰n 3:sta, koska sill‰ osoitetaan ensimm‰ist‰ argumenttia
argloop:	
 	mov	eax, dword [ebp + 4 * edx] ;EAXiin ensimm‰inen array-argumentti
	movzx	ebx, byte [eax] ;siit‰ ensimm‰inen tavu EBX:‰‰n
	cmp	ebx, 45		;onko tavun arvo -:n ASCII-koodi, jos ovat samat ZF on asetettu
	jz	sw_loytyi	;jos ZF p‰‰ll‰, haaraudu
	
	mov	[fname], eax 	;fnameen ensimm‰isen elementin osoite argv:st‰ (joka on filename, jos t‰ss‰ kohtaa ollaan)
	;; v‰hennet‰‰n EDX-kountterista - 2 ja verrataan argm:iin. Jos samat, looppi loppuu. Muuten kountteri kasvaa
	mov	eax, edx
	sub	eax, 2
	movzx	ecx, byte [argm]
	cmp	ecx, eax
	jz	sw_end
	add	edx, 1
	jmp	short argloop
	;; eax = edx-2, jos eax = argm, lopetetaan arg-looppi, muuten argloopin alkuun
sw_loytyi:
	movzx	ebx, byte [eax+1] ;siit‰ toinen tavu EBX:‰‰n
	cmp	ebx, 97		  ;jos EBX on a:n ASCII-koodi
	jnz	sw_end		  ;jos ei ollut (ZF ei asetettu)
	mov	byte [ascii], 1	  ;[ascii] p‰‰lle, jos oli
	;; sama loopin loppu -kysely kuin edell‰
	mov	eax, edx
	sub	eax, 2
	movzx	ecx, byte [argm]
	cmp	ecx, eax
	jz	sw_end
	add	edx, 1
	jmp	short argloop
sw_end:
	
	;; onko argumentti switch?
	mov     edx, 1	        ;viestin pituus
	mov	ecx, eax
	mov     ebx, 1          ;file descriptor (stdout)
	mov     eax, 4          ;system call number (sys_write)
	int     0x80            ;int = software interrupt
	
	;; muuttujien alustusta
	mov	byte [vkount], 0
	mov	byte [rkount], 0

	;; sys_stat, onko tiedosto olemassa
	mov	ebx, [fname]
	mov	ecx, statbuf
	mov	eax, 18
	int	0x80

	mov	ebx, -75	; jostain syyst‰ syscallin onnistunut kutsu palauttaakin EAX:‰‰n -75
	cmp	eax, ebx	
	jnz	eitied		; lopetetaan, jos tiedostoa ei lˆydy
	
	;; sys_open, avataan tiedosto
	mov	ebx, [fname]	; EBX:‰‰n syscallin ensimm‰inen argumentti
	mov	ecx, O_RDONLY	; syscallin toinen argumentti
	mov	eax, 5		; 5 = sys_openin numero
	int	0x80		; keskeytys 0x80 (128) kutsuu syscallin, jonka numero otetaan EAXista
	mov	[fd], eax	; sys_call palauttaa file descriptorin EAX:aan ja se laitetaan sille kuuluvalle paikalleen
	
lukeminen:	
	;; sys_read, luetaan
	mov	ebx, [fd]	; FD argumentiksi
	mov	ecx, buffer	; bufferin osoite argumentiksi
	mov	edx, 1		; size_t count -argumentti eli paljonko luetaan
	mov	eax, 3		; sys_readin numero EAX:‰‰n
	int	0x80

	;; ollaanko jo tiedoston lopussa
	mov	ebx, 0
	cmp	eax, ebx	; jos sys_read palauttaa nollan EAX:‰‰n, ollaan tiedoston lopussa
	jnz	ei_lopu		; CMP laittaa ZF:n p‰‰lle, jos numerot ovat samat, jos se ei ole p‰‰ll‰, hyp‰t‰‰n jatkoon
	;; jos ascii on p‰‰ll‰, tehd‰‰n viel‰ viimeinen ascii-rivi
	mov	al, 1
	cmp	al, [ascii]
	jz	vika_arivi
	;; jos ascii = 2 eli ollaan tekem‰ss‰ vikaa ascii-rivi‰, laitetaan loppuun viel‰ pystyviiva
	mov	al, 2
	cmp	al, [ascii]
	jnz	lopetus
	mov     edx, 1	        ;viestin pituus
	mov     ecx, pviiva
	mov     ebx, 1          ;file descriptor (stdout)
	mov     eax, 4          ;system call number (sys_write)
	int     0x80            ;int = software interrupt
	jmp 	lopetus
ei_lopu:
	;; ollaanko kirjoittamassa ASCIIta (jos rkount on yli 15)
	mov	bl, 15
	cmp	bl, [rkount]
	jc	k_ascii
	
	;; vertailut alkaa
	;; Onko bufferissa oleva arvo alle 16?
	mov	ebx, 15
	cmp	ebx, [buffer]	;jos vasen on pienempi, CF p‰‰lle, ZF pois
	jc	suurempi	;EBX pienempi eli luku yli 16 ja hyp‰t‰‰n
	mov	byte [ekaa], 48	;Kun ensimm‰inen heksa ei ole k‰ytˆss‰, siihen voi laittaa suoraan nollan (eli 48 ASCIIna)
	movzx	edx, byte [buffer] ;T‰m‰ on eritt‰in t‰rke‰. Pelkk‰ MOV j‰tt‰‰ EDX:‰‰n satunnaisia arvoja ("bufferista"), MOVZX nollaa sen
	call 	konversio	;laittaa stackiin seuraavan rivin osoitteen
	mov	[tokaa], edx
	jmp	short vertloppu
suurempi:			;Jos arvo on yli 15
	xor	edx, edx	;DIV jakaa EDX:EAX:n operaattorilla, joten t‰m‰n voisi XORrata varmuuden vuoksi
	movzx	eax, byte [buffer]
	mov	ebx, 16
	div	ebx		;Jakaa 32-bittisen -> osam‰‰r‰ (ensimm‰inen heksaluku) menee EAX:‰‰n ja jakoj‰‰nnˆs (toinen heksaluku) EDX:‰‰n
	call	konversio	;Konversio k‰ytt‰‰ EDX:‰‰, jonne jakoj‰‰nnˆs meni, joten hoidetaan ensin toinen heksaluku
	mov	[tokaa], edx
	mov	edx, eax   	;Osam‰‰r‰ on EAX:ss‰, joten nyt se EDX:‰‰n konversiota varten
	call 	konversio	;laittaa stackiin seuraavan rivin osoitteen stackiin
	mov	[ekaa], edx
	jmp	short vertloppu
konversio:			;t‰m‰ kohta muuttaa 0-15 desimaaliarvon heksan ASCII-arvoksi, EDX:ss‰ on se arvo, mik‰ konvertoidaan
	mov	ebx, 9
	cmp	ebx, edx	;jos vasen arvo on pienempi (eli arvo on suurempi kuin 9), ZF on pois p‰‰lt‰ ja CF on p‰‰ll‰
	jc	yliysi		;jc hypp‰‰, vain jos CF on p‰‰ll‰ (muut CMP:n tulokset eiv‰t laita CF:‰‰ p‰‰lle)
	add	edx, 48		;kun lis‰t‰‰n 0-9 -kokoiseen arvoon 48, tulee ASCIIn 0-9 (48-57)
	ret			;palaa sinne, mist‰ on tultu (stackissa pit‰‰ olla ilmeisesti p‰‰llimm‰isen‰ se)
yliysi:	
	add	edx, 55		;Kun lis‰t‰‰n kymmenlukuihin 10-15 luku 55, saadaan ASCII-arvot A-F (65-70)
	ret			

vertloppu:	
	;; Heksojen printtaus
	mov     edx, 1	        ;viestin pituus
	mov     ecx, ekaa	;itse viesti (pointteri)
	mov     ebx, 1          ;file descriptor (stdout)
	mov     eax, 4          ;system call number (sys_write)
 	int     0x80            ;int = software interrupt
	
	mov     ecx, tokaa	;itse viesti (pointteri)
	mov     eax, 4          ;system call number (sys_write)
	int     0x80            ;int = software interrupt
	inc	byte [rkount]
	;; vkountin nollaus ja v‰lilyˆnnin printtaus tuleekin samaan aikaan. Jos vkount on 1 -> printtaa ja nollaa
	inc	byte [vkount]
	mov	bl, 1
	cmp	bl, [vkount]
	jc	valprint
	jmp	lukeminen
valprint:	
	mov	byte [vkount], 0
	mov     edx, 1	        ;viestin pituus
	mov     ecx, valil	;itse viesti (pointteri)
	mov     ebx, 1          ;file descriptor (stdout)
	mov     eax, 4          ;system call number (sys_write)
	int     0x80            ;int = software interrupt

	;; vaihtuuko rivi
	mov	bl, 15
	cmp	bl, [rkount]
	jc	a_vai_v
	jmp	lukeminen
a_vai_v:
	;; ASCIIta vai rivinvaihtoa
	mov	al, 1
	cmp	al, [ascii]
	jnz	vaihto
	jmp	short k_ascii
vaihto:	
	call	rivinvaihto
	jmp 	lukeminen
k_ascii:
	;; t‰ss‰ pit‰‰ tehd‰ uusi rivinvaihdon tarkistus
	;; sitten katsotaan se ascii-arvo ...pit‰‰kˆ k‰ytt‰‰ lseeki‰ vai pit‰‰kˆ n‰m‰ ASCIIt lukea aiemmin
	;; jos rkount on ensimm‰isess‰ ascii-merkiss‰, niin lseek rivin alkuun. jos taas lopussa, kelataan se seuraavalle riville
	mov	bl, 16		;8 on ilmeisesti ensimm‰inen ASCII-arvo
	cmp	bl, [rkount]
	jc	ei_eka		;jos ei ole samat, ei olla ensimm‰isess‰ ASCII-arvossa
	;; jos ollaan ensimm‰isess‰ ASCII-arvossa, sitten printataan myˆs v‰lilyˆnti ja pystyviiva, siirret‰‰n lseek-offsettia ja lopuksi merkinkirjoituskohtaan

	mov	ebx, [fd]
	mov	ecx, -16     	;offset
	mov	edx, 1		;1 = SEEK_CUR = aloitetaan nykyisest‰ kohdasta
	mov     eax, 19         ;19 = lseek
	int     0x80
k_ascii2:	
	;; T‰h‰n v‰liin sysread, samanlainen oli aiemminkin
	mov	ebx, [fd]	; FD argumentiksi
	mov	ecx, buffer	; bufferin osoite argumentiksi
	mov	edx, 1		; size_t count -argumentti eli paljonko luetaan
	mov	eax, 3		; sys_readin numero EAX:‰‰n
	int	0x80

	mov     edx, 1	        ;v‰lilyˆnnin printtaus, esiintyy aiemminkin koodissa
	mov     ecx, valil
	mov     ebx, 1
	mov     eax, 4
	int     0x80

	mov     edx, 1	        ;viestin pituus
	mov     ecx, pviiva
	mov     ebx, 1          ;file descriptor (stdout)
	mov     eax, 4          ;system call number (sys_write)
	int     0x80            ;int = software interrupt

	jmp	as_vai_ps

ei_eka:
	;; ollaanko jo viimeisen kohdan yli?
	mov	bl, 31		;oli 24
	cmp	bl, [rkount]
	jnc	as_vai_ps

	;; rivin loppuun tuleva pystyviiva
	mov     edx, 1	        ;viestin pituus
	mov     ecx, pviiva
	mov     ebx, 1          ;file descriptor (stdout)
	mov     eax, 4          ;system call number (sys_write)
	int     0x80            ;int = software interrupt

	;; ollaanko lopussa

	mov	al, 2
	cmp	al, [ascii]
	jz	lopetus
	;; lseekin palautus
	mov	ebx, [fd]
	mov	ecx, -1
	mov	edx, 1
	mov     eax, 19
	int     0x80

	call	rivinvaihto
	jmp	lukeminen

as_vai_ps:
	;; sen j‰lkeen kysyt‰‰n, onko arvo yli 127. jos kyll‰, piirret‰‰n piste, muuten asciin piirtoon
	inc	byte [rkount]
	mov	ebx, 126
	cmp	ebx, [buffer]	;jos vasen on pienempi, CF p‰‰lle, ZF pois
	jc	kpiste		;EBX pienempi eli luku yli 127 ja hyp‰t‰‰n

	mov	ebx, 31
	cmp	[buffer], ebx
	jc	kpiste		
	;; printataan ASCII
	mov     edx, 1	        
	mov     ecx, buffer
	mov     ebx, 1          
	mov     eax, 4          
	int     0x80
	jmp	lukeminen

kpiste:
	mov     edx, 1	        
	mov     ecx, piste
	mov     ebx, 1          
	mov     eax, 4          
	int     0x80            
	jmp	lukeminen
	
rivinvaihto:
	mov 	edx, 1
	mov	ebx, 1		;fd (stdout)
	mov	eax, 4
	mov	ecx, rivinv
	int	0x80		
	mov	byte [rkount], 0
	ret

vika_arivi:			;viimeinen ascii-rivi
	mov	byte [ascii], 2	;merkiksi, ett‰ lopetetaan t‰m‰n rivin j‰lkeen

	;; Jos heksarivi on t‰yspitk‰, ei tarvitse lis‰ill‰ mit‰‰n v‰lilyˆntej‰ en‰‰:
	mov	bl, 32
	cmp	bl, [rkount]
	jz	lukeminen

	;; v‰lilyˆntien lis‰ys ennen ASCII-rivi‰ viimeisen heksa-rivin lyhyemm‰n pituuden takia
	mov	byte [vkount], 1 ;samaa vkount-muuttujaa voi k‰ytt‰‰ t‰ss‰, arvo 1, koska juuri ennen pystyviivaa tulee aina ylim‰‰r‰inen v‰lilyˆnti
	mov	al, [rkount]
	mov	bl, 16
	sub	bl, al		;16 - rkount, saadaan montako rkountia puuttuu t‰ydest‰ rivist‰
	movzx	ax, [rkount]	;DIV jakaa AX:n (jos operandi on 8-bittinen)
	mov	cl, 2
	div	cl		;osam‰‰r‰ AL:‰‰n ja jakoj‰‰nnˆs AH:hon
	cmp	ah, 0		;jos jakoj‰‰nnˆs 0, luku on parillinen
	jz	paril		;jos parillinen
	add	byte [vkount], 2
	sub	bl, 1
	jmp	short jatko
paril:
	add	byte [vkount], 4
	sub	bl, 2
jatko:
	movzx	ax, bl		;muunneltu p-rkount ax:‰‰n
	div	cl		;jaetaan taas kahdella
	mov	bl, al		;siirret‰‰n, koska AL/AX/EAX k‰ytet‰‰n MULissa
	mov	al, 5		;MUL kertoo tavun kokoiset AL:ll‰
	mul	bl		;tulos DX:AX:‰‰n
	;; (LOOP-k‰sky ei onnistu, koska se k‰ytt‰‰ ECX:‰‰, jota tarvitaan loopissa syscalliin)
	;; myˆs EBX:‰‰ k‰ytet‰‰n, joten AL pit‰isi siirt‰‰ esim. vkountiin:
	add	byte [vkount], al ;ei tarvitse muuntaa erikseen AX:‰‰ AL:ksi, se on jo siell‰ jos se kerran mahtuu AL:‰‰n
looppi:
	mov     edx, 1	        ;viestin pituus
	mov     ecx, valil	;itse viesti (pointteri)
	mov     ebx, 1          ;file descriptor (stdout)
	mov     eax, 4          ;system call number (sys_write)
	int     0x80            ;int = software interrupt
	dec	byte [vkount]
	mov	al, 0
	cmp	[vkount], al
	jnz	looppi
	movzx	ecx, byte [rkount]
	neg	ecx
	mov	ebx, [fd]
	mov	edx, 1		;1 = SEEK_CUR = aloitetaan nykyisest‰ kohdasta
	mov     eax, 19         ;19 = lseek
	int     0x80
	mov	byte [rkount], 16
	jmp	k_ascii2

eitied:	
	mov     edx, len2-1	
	mov     ecx, ei_loyd	
	mov     ebx, 1          
	mov     eax, 4          
	int     0x80            
	
lopetus:	
	call	rivinvaihto	
	mov	eax, 1		; exit-syscall
	int 	0x80
