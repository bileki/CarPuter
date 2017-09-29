Se usoDoPID está ativo Então

	Se velocidadeAtual está numa faixa constante Então
		IncrementaTempoDeIterações
	SeNão
		DefineFaixaDeVelocidade(velocidadeAtual)
		ZeraTempoDeIterações
	FimSe

	Se manteve faixa de velocidade constante Então
		LigaLED
		DefineTargetVelocidade(velocidadeAtual)
		DefineServoIgualPosiçãoDoCabo
		Se passou algumas iterações Então
			HabilitaUsoDoPID
		FimSe
	FimSe

SeNão

	Se aceleração ultrapassar target Então
		DesabilitaUsoDoPID
		DesligaLED
		ZeraTemposDeIterações
		DefineFaixaDeVelocidade(velocidadeAtual)
	FimSe

FimSe






void IA() {
  // Caso o cálculo do PID não esteja habillitado
  if (!flagPID) {
    // Verifica se a velocidade está dentro de uma faixa constante
    if ((velocity < (velocityBand + FAIXA)) && (velocity > (velocityBand - FAIXA))) {
      // Incrementa as interações
      countInteractions++;
    } else {
      // Caso a velocidade saia da faixa constante, redefine-se a faixa e zera as interações
      countInteractions = 0;
      velocityBand = velocity;
    }
    // Caso o motorista mantenha a faixa constante de velocidade por 300 interações
    if (countInteractions > INTERACTIONS) {
      // Liga-se o LED
      digitalWrite(LED, HIGH);
      flagLight = true;
      // Redefine-se o target da velocidade
      targetVelocity = velocity;
      // Define o valor do servo com o último valor enviad ao servo
      servo = pwm;     
      	  // Espera 3 interações para ativar o uso do PID
	      if (count > 3) { 
	        flagActivatePID = true;
	        flagPID = true;
	      } else {
	        // Caso contrário, zera a soma do fator integrativo, pois o erro acumulado pode ser muito grande e atrapalhar a primeira correção do PID
	        sum = 0;
	      }
	      // Incrementa a contagem 
	      count++;   
    }
  } else {
    // Caso o cáculo do PID esteja habilitado e o motorista ultrapassar o target da velocidade
    if (throttle > targetVelocity) {
      // Desativa o cálculo e o uso do PID, apaga o LED, zera os contadores e redefine a faixa de velocidae
      flagPID = false;
      flagActivatePID = false;
      digitalWrite(LED, LOW);
      flagLight = false;
      countInteractions = 0;
      count = 0;
      velocityBand = velocity;
    }
  }
}