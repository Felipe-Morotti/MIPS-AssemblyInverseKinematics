.data
	# Mensagens
	msgL1: .asciiz "Dê o comprimento de L1: "
	msgL2: .asciiz "Dê o comprimento de L2: "
	msgPx: .asciiz "Dê a coordenada X do end-affector: "
	msgPy: .asciiz "Dê a coordenada Y do end-affector: "
	msgT1: .asciiz "O ângulo (T1) da primeira junta, em graus, é: "
	msgT2: .asciiz "O ângulo (T2) da segunda junta, em graus, é: "
	msgErr: .asciiz "\nErro: Alvo fora de alcance."
	
	# Constantes Matemáticas
	zero: .float 0.0
	one: .float 1.0
	neg_one: .float -1.0
	two: .float 2.0
	pi: .float 3.14159265
	two_pi: .float 6.28318530718  #2*pi
	meio_pi: .float 1.57079632
	rad_deg: .float 57.29578
	max_iter: .word 50
	
	# Variáveis Globais
	L1: .float 0.0
	L2: .float 0.0
	Px: .float 0.0
	Py: .float 0.0
	dist_sq: .float 0.0
	dist_linear: .float 0.0

.text
.globl main

main:
	jal getInputFunction
	jal calculateDistances
	jal verificaAlcance # Encerra se inalcançável
	
	# --- CÁLCULO DE THETA 2 ---
	jal calculaTheta2
	mov.s $f28, $f12 # $f28 = Theta2 (rad)
	
	# --- CÁLCULO DE THETA 1 ---
	jal calculaTheta1
	mov.s $f29, $f12 # $f29 = Theta1 (rad)
	
	# Normaliza Theta1
    	mov.s $f12, $f29
    	jal normalize
    	mov.s $f29, $f12
    	# Normaliza Theta2
    	mov.s $f12, $f28
    	jal normalize
    	mov.s $f28, $f12

	# --- IMPRESSÃO DOS RESULTADOS ---
	# T1
	li $v0, 4
	la $a0, msgT1
	syscall
	mov.s $f12, $f29
	jal print_as_degrees
	
	# Quebra de linha (opcional, mas bom pra visualização)
	li $a0, 10
	li $v0, 11
	syscall
		
	# T2
	li $v0, 4
	la $a0, msgT2
	syscall
	mov.s $f12, $f28
	jal print_as_degrees
	
	li $v0, 10 # Exit
	syscall

# ==========================================================
# FUNÇÕES DE ENTRADA E DISTÂNCIA
# ==========================================================

getInputFunction:
	# L1
	li $v0, 4
	la $a0, msgL1
	syscall
	li $v0, 6
	syscall
	s.s $f0, L1
	# L2
	li $v0, 4
	la $a0, msgL2
	syscall
	li $v0, 6
	syscall
	s.s $f0, L2
	# Px
	li $v0, 4
	la $a0, msgPx
	syscall
	li $v0, 6
	syscall
	s.s $f0, Px
	# Py
	li $v0, 4
	la $a0, msgPy
	syscall
	li $v0, 6
	syscall
	s.s $f0, Py
	jr $ra

calculateDistances:
	l.s $f20, Px
	l.s $f21, Py
	mul.s $f22, $f20, $f20
	mul.s $f23, $f21, $f21
	add.s $f14, $f22, $f23    # d^2
	s.s $f14, dist_sq
	sqrt.s $f15, $f14         # d
	s.s $f15, dist_linear
	jr $ra

verificaAlcance:
	l.s $f4, L1
	l.s $f5, L2
	l.s $f15, dist_linear
	
	# Teste Máximo: d > L1 + L2
	add.s $f6, $f4, $f5
	c.lt.s $f6, $f15
	bc1t erro_alcance
	
	# Teste Mínimo: d < |L1 - L2|
	sub.s $f6, $f4, $f5
	abs.s $f6, $f6
	c.lt.s $f15, $f6
	bc1t erro_alcance
	jr $ra

erro_alcance:
	li $v0, 4
	la $a0, msgErr
	syscall
	li $v0, 10
	syscall

# ==========================================================
# CINEMÁTICA INVERSA
# ==========================================================

calculaTheta2:
	# Lei dos Cossenos: cos(T2) = (d^2 - L1^2 - L2^2) / (2*L1*L2)
	addi $sp, $sp, -4
	sw $ra, 0($sp)
	
	l.s $f14, dist_sq
	l.s $f4, L1
	l.s $f5, L2
	mul.s $f4, $f4, $f4       # L1^2
	mul.s $f5, $f5, $f5       # L2^2
	
	sub.s $f6, $f14, $f4
	sub.s $f6, $f6, $f5       # Numerador
	
	l.s $f7, L1
	l.s $f8, L2
	l.s $f9, two
	mul.s $f10, $f7, $f8
	mul.s $f10, $f10, $f9     # Denominador
	
	div.s $f12, $f6, $f10     # cos(T2)
	jal acos
	
	neg.s $f12, $f12          # torna negativo para solução cotovelo para baixo  #Alteração DeepSeek
    	mov.s $f28, $f12
    	lw $ra, 0($sp)
    	addi $sp, $sp, 4
    	jr $ra

calculaTheta1:
	# Theta1 = Atan2(Py, Px) - Acos( (d^2 + L1^2 - L2^2) / (2*d*L1) )
	addi $sp, $sp, -4
	sw $ra, 0($sp)
	
	# --- PARTE A: CALCULAR ATAN2(Py, Px) BLINDADO ---
	l.s $f20, Px
	l.s $f21, Py
	l.s $f8, zero
	
	# Verifica se Px == 0 para evitar divisão por zero
	c.eq.s $f20, $f8
	bc1t trata_px_zero
	
	# Caso normal: Px != 0
	div.s $f0, $f21, $f20     # Py / Px
	jal robusta_atan
	mov.s $f27, $f12          # $f27 guarda o ângulo base
	
	# Correção de Quadrante (Se Px < 0, somar PI)
	l.s $f20, Px
	l.s $f8, zero
	c.lt.s $f20, $f8
	bc1f calc_parte_b
	l.s $f9, pi
	add.s $f27, $f27, $f9
	j calc_parte_b

trata_px_zero:
	# Se Px=0 e Py>0 -> 90 graus. Se Py<0 -> -90 graus.
	l.s $f27, meio_pi
	l.s $f21, Py
	l.s $f8, zero
	c.lt.s $f21, $f8
	bc1f calc_parte_b
	neg.s $f27, $f27          # vira -90 se y negativo

calc_parte_b:
	# --- PARTE B: LEI DOS COSSENOS PARA ÂNGULO INTERNO ---
	l.s $f14, dist_sq
	l.s $f4, L1
	l.s $f5, L2
	l.s $f15, dist_linear
	mul.s $f4, $f4, $f4       # L1^2
	mul.s $f5, $f5, $f5       # L2^2
	
	add.s $f6, $f14, $f4
	sub.s $f6, $f6, $f5       # Numerador
	
	l.s $f7, two
	l.s $f8, L1
	mul.s $f9, $f7, $f15
	mul.s $f9, $f9, $f8       # Denominador
	
	div.s $f12, $f6, $f9      # cos(phi)
	jal acos
	
	# Resultado Final: Atan2 - Acos
	add.s $f12, $f27, $f12 #Alteração DeepSeek
	
	lw $ra, 0($sp)
	addi $sp, $sp, 4
	jr $ra

# ==========================================================
# FUNÇÕES MATEMÁTICAS ROBUSTAS
# ==========================================================

acos:
	# Recebe cos(x) em $f12, retorna arccos(x) em $f12
	addi $sp, $sp, -4
	sw $ra, 0($sp)
	
	# 1. CLAMPING (Segurança contra arredondamento tipo 1.0000001)
	l.s $f1, one
	l.s $f2, neg_one
	
	c.le.s $f12, $f1       # Se x <= 1.0
	bc1t check_min_acos
	mov.s $f12, $f1        # Força 1.0
	j start_acos_calc
check_min_acos:
	c.lt.s $f12, $f2       # Se x < -1.0
	bc1f start_acos_calc
	mov.s $f12, $f2        # Força -1.0

start_acos_calc:
	mov.s $f10, $f12       # Salva input original (para checar sinal depois)
	
	# Fórmula: atan( sqrt(1-x^2) / x )
	mul.s $f1, $f12, $f12  # x^2
	l.s $f2, one
	sub.s $f1, $f2, $f1    # 1 - x^2
	
	# Proteção sqrt negativo
	l.s $f8, zero
	c.lt.s $f1, $f8
	bc1t force_zero_sqrt
	sqrt.s $f1, $f1        # sqrt(1-x^2)
	j check_div_zero
force_zero_sqrt:
	mov.s $f1, $f8

check_div_zero:
	# Se x é muito próximo de 0, atan é 90 graus
	abs.s $f9, $f12
	l.s $f8, zero
	c.eq.s $f9, $f8
	bc1t ret_90_degs
	
	div.s $f0, $f1, $f12   # argumento do atan
	jal robusta_atan
	
	# Correção para Arccos Obtuso (2º Quadrante)
	# Se input original ($f10) era negativo, atan deu negativo.
	# Arccos range é [0, PI]. Se negativo, somar PI.
	l.s $f8, zero
	c.lt.s $f10, $f8
	bc1f end_acos
	l.s $f9, pi
	add.s $f12, $f12, $f9
	j end_acos

ret_90_degs:
	l.s $f12, meio_pi
end_acos:
	lw $ra, 0($sp)
	addi $sp, $sp, 4
	jr $ra

robusta_atan:
	# Recebe $f0, retorna atan($f0) em $f12
	# Usa identidade: atan(x) = pi/2 - atan(1/x) para x > 1
	addi $sp, $sp, -4
	sw $ra, 0($sp)
	
	l.s $f1, one
	abs.s $f2, $f0
	c.le.s $f2, $f1
	bc1t call_series_atan   # Se |x| <= 1, calcula direto
	
	# Se |x| > 1, inverte
	div.s $f5, $f1, $f0     # f5 = 1/x
	
	# Checa sinal original
	l.s $f8, zero
	c.lt.s $f0, $f8
	bc1t handle_large_neg
	
	# Caso Positivo Grande: pi/2 - atan(1/x)
	mov.s $f0, $f5
	jal atan_series
	l.s $f1, meio_pi
	sub.s $f12, $f1, $f12
	j end_robusta

handle_large_neg:
	# Caso Negativo Grande: -pi/2 - atan(1/x)
	mov.s $f0, $f5
	jal atan_series
	l.s $f1, meio_pi
	neg.s $f1, $f1
	sub.s $f12, $f1, $f12
	j end_robusta

call_series_atan:
	jal atan_series
end_robusta:
	lw $ra, 0($sp)
	addi $sp, $sp, 4
	jr $ra

atan_series:
	# Série de Taylor para atan(x) onde |x| <= 1
	# atan(x) = x - x^3/3 + x^5/5 ...
	mov.s $f12, $f0        # Acumulador (começa com x)
	mul.s $f3, $f0, $f0    # f3 = x^2 (constante multiplicativa)
	mov.s $f2, $f0         # f2 = termo atual (começa com x)
	
	l.s $f4, neg_one       # f4 = sinal (alterna)
	
	lw $t1, max_iter
	li $t2, 1              # contador
	li $t3, 3              # denominador (3, 5, 7...)

loop_atan:
	bgt $t2, $t1, end_series
	
	mul.s $f2, $f2, $f3    # termo *= x^2 (eleva potência)
	
	mtc1 $t3, $f5
	cvt.s.w $f5, $f5       # f5 = float(denominador)
	
	div.s $f6, $f2, $f5    # f6 = termo / denominador
	mul.s $f6, $f6, $f4    # f6 = f6 * sinal
	
	add.s $f12, $f12, $f6  # soma ao total
	
	neg.s $f4, $f4         # inverte sinal
	addi $t2, $t2, 1
	addi $t3, $t3, 2
	j loop_atan

end_series:
	jr $ra
	
# Função para normalizar ângulo em $f12 para o intervalo [-π, π]
normalize:
 	l.s $f1, pi
    	l.s $f2, two_pi
    	l.s $f3, zero
norm_gt:
    	c.le.s $f12, $f1
    	bc1t norm_lt
    	sub.s $f12, $f12, $f2
    	j norm_gt
norm_lt:
    	neg.s $f4, $f1
    	c.lt.s $f12, $f4
    	bc1f end_norm
    	add.s $f12, $f12, $f2
    	j norm_lt
end_norm:
    	jr $ra
	
print_as_degrees:
	l.s $f1, rad_deg
	mul.s $f12, $f12, $f1
	li $v0, 2
	syscall
	jr $ra
