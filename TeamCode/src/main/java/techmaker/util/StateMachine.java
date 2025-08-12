package techmaker.util;

/**
 * Enumeração que define todos os estados de operação do robô.
 * Esta versão combina os estados originais da equipa com os novos estados
 * necessários para a máquina de estados refatorada e orientada a eventos.
 */
public enum StateMachine {
    // --- Estados Originais ---
    IDLE,
    TRAVELLING,
    START_INTAKE,
    MEDIUM_INTAKE,
    INTAKE_AVANCE,
    INTAKE_DETECTING,
    REVERTING_INTAKE,
    INTAKING,
    RETURNING_INTAKE,
    INTAKE_RETURNED,
    CLAW_SAMPLE,
    CLAW_SPECIMENT,
    DELIVER_SAMPLE,
    DELIVERY_SPECIMENT,
    CLAW_RETRACT,
    AUTO_CYCLE_START,
    AUTO_INTAKING,
    AUTO_GRAB,
    AUTO_RAISE,
    START_INTAKE_MEDIUM,
    INTAKE_MEDIUM_RETURNED,

    // --- NOVOS ESTADOS para a Lógica Orientada a Eventos ---

    // Estado intermediário para configurar o intake antes de começar a capturar.
    INTAKE_SETUP,

    // Estados para iniciar o ciclo de pontuação em diferentes alturas.
    SCORE_LOW,
    SCORE_MEDIUM,
    SCORE_HIGH,

    // Estados de espera que aguardam a conclusão de uma ação de hardware.
    WAITING_FOR_LIFT,
    WAITING_FOR_RETURN;
}
