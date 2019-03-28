
	;; assemble:
	;; nasm -f elf hexdumper.asm
	;; ld -m elf_i386 -o hexdumper hexdumper.o
	;; usage:
	;; ./hexdumper filename
	

	%define	O_RDONLY	00 ;syˆtet‰‰n flags 0:na, mutta koodi on selke‰mp‰‰, kun siin‰ k‰ytt‰‰ tuota flagsin kirjainmuotoa, joka siis vastaa nollaa
	
segment .data
;;; 	fname	db 'testfile', 0 ;polku/tiedostonimi avattavaan tiedostoon
	valil	db 32		 ;32 = valilyˆnnin ASCII-arvo (dec.)
	rivinv	db 10		 ;10 = rivinvaihdon ASCII-arvo (dec.)
	ei_args db 'Virhe: Et antanut tiedoston nime‰', 0
	len     equ $ - ei_args

segment .bss
	;; Oleellista, ett‰ n‰iss‰ muistipaikoissa ei ole sis‰llˆn koko vaan koko tulee sen mukaan, millaisilla rekistereill‰ niit‰ k‰ytet‰‰n
	loppu	resb 1		;boolelainen muuttuja, joka kertoo, ollaanko luettavan tiedoston lopussa
	ascii	resb 1		;boolelainen muuttuja, n‰ytet‰‰nkˆ ASCIIt
	vkount	resb 1		;kountteri v‰lilyˆnneille, jotka tulee joka nelj‰nnelle tavulle
	rkount	resb 1		;kountteri riveille
	argm	resb 1		;argumenttien m‰‰r‰

	fname	resd 1		;pointteri tiedostonnimeen osoittavaan argumenttiin
	buffer	resd 1		;tavun kokoinen bufferi, luetaan tiedostoa tavu kerrallaan
	fd	resd 1		;file descriptorin paikka, koko on double, koska vaikka FD itse on vain byten, muistipaikkaa operoidaan EAX:ill‰ (32-bit = double)
	ekaa	resd 1		;ensimm‰inen printattava heksa
	tokaa	resd 1		;toinen printattava heksa
	
segment .text
	global _start
_start:	
	;; ESP ja EBP (?) talteen ja EBP osoittamaan komentoriviargumentteja, ebp + 4 = argumentit
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
	xor	eax, eax
	mov	eax, dword [ebp + 4]
	;; 	movzx	eax, dword [ebp + 4]
	sub	eax, 1 		;t‰h‰n tulee aina yksi suurempi luku kuin mik‰ on argumentti, joten --
	mov	[argm], eax
	mov	edx, 3		;t‰st‰ kountteri k‰sitellyille argumenteille, v‰hennet‰‰n aina kaksi, niin on kountteriarvo, muuten l‰hdet‰‰n 3:sta, koska sill‰ osoitetaan ensimm‰ist‰ argumenttia
argloop:	
	xor	eax, eax
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
	cmp	ebx, 99		  ;jos EBX on c:n ASCII-koodi
	jnz	sw_end		  ;jos ei ollut (ZF ei asetettu)
	mov	byte [ascii], 1	  ;[ascii] p‰‰lle, jos oli
debug:	
	;; sama loopin loppu -kysely kuin edell‰
	mov	eax, edx
	sub	eax, 2
	movzx	ecx, byte [argm]
	cmp	ecx, eax
	jz	sw_end
	add	edx, 1
	jmp	short argloop
sw_end:
	;; muuttujien alustusta
	mov	byte [loppu], 0
	mov	byte [vkount], 0
	mov	byte [rkount], 0
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
	mov	byte [loppu], 1	; loppu = 1 eli lopetellaan, kun on saatu kirjoitettua j‰ljell‰ olevat
ei_lopu:	

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
	movzx	eax, byte [buffer] ;Move with zero extension, kuten yll‰
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
	;; Per‰tt‰isist‰ syscalleista voi ottaa nuo turhat kohdat pois
	;; Heksojen printtaus
	mov     edx, 1	        ;viestin pituus
	mov     ecx, ekaa	;itse viesti (pointteri)
	mov     ebx, 1          ;file descriptor (stdout)
	mov     eax, 4          ;system call number (sys_write)
 	int     0x80            ;int = software interrupt
	
	mov     ecx, tokaa	;itse viesti (pointteri)
	mov     eax, 4          ;system call number (sys_write)
	int     0x80            ;int = software interrupt

	;; vkountin nollaus ja v‰lilyˆnnin printtaus tuleekin samaan aikaan. Jos vkount on 1 -> printtaa ja nollaa
	clc
	inc	byte [vkount]
	mov	bl, 1
	cmp	bl, [vkount]
	jc	valprint
	jmp	short valloppu
valprint:	
	mov	byte [vkount], 0
	mov     edx, 1	        ;viestin pituus
	mov     ecx, valil	;itse viesti (pointteri)
	mov     ebx, 1          ;file descriptor (stdout)
	mov     eax, 4          ;system call number (sys_write)
	int     0x80            ;int = software interrupt

	;; vaihtuuko rivi
	inc	byte [rkount]	
	mov	bl, 8
	clc
	cmp	bl, [rkount]
	jc	vaihto
	jmp	short valloppu
vaihto:	
	call	rivinvaihto
	jmp 	short valloppu
rivinvaihto:
	mov 	edx, 1
	mov	ebx, 1		;fd (stdout)
	mov	eax, 4
	mov	ecx, rivinv
	int	0x80		
	mov	byte [rkount], 0
	ret
	
valloppu:
	;; Tarkistetaan tiedoston loppumisesta kertova muuttuja
	mov	al, 1
	cmp	al, [loppu]
	jnz	lukeminen	;Jos loppu-muuttuja ei ole p‰‰ll‰, hyp‰t‰‰n takaisin lukemaan (jos erisuuret -> ZF pois, JNZ = hypp‰‰ jos ZF pois)

lopetus:	
	call	rivinvaihto	
	mov	eax, 1		; exit-syscall
	int 	0x80

