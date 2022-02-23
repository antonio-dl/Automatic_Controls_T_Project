% Controllo dell'assetto di un Drone Planare
% Traccia 1c
% Gruppo U
%
% Antonio De Luca
% Grilli Giacomo
% Marco Esopi
% Sara Abukar Adawe
% 30/11/2020
%

%% Inizio
clear %all
close all
clc

% Caratteristiche dell'impianto
Fv = -12 ;      % forza del vento laterale
Id = 0.3 ;      % momento di inerzia del drone rispetto all'asse di rotazione
beta = 0.3;     % coefficiente di atrito dinamico
a = 0.25;       % distanza fra baricentro e applicazione della forza dei motori
omega_e = 0;    % condizione di equilibrio (velocita' angolare nulla)
theta_e = -pi/6; % condizione di equilibrio (angolo theta imposto)


syms x1 x2 u;

f_1 = x2;
f_2 = a/2*sin(x1)*Fv/Id -beta/Id*x2 + a/Id * u;
% Esperimenti INIZIO
if 0
    AA = [ f_1;f_2];
    
    J_AA = jacobian(AA,[x1;x2])
end
%Esperimenti FINE

% Calcolo delle condizioni di equilibrio
% f_1 = 0  -->  x2 = 0
% f_2 = 0  -->  -beta*x2 + a/2*sin(x1)*Fv + a*u = 0 -->
%               1/2*sin(x1)*Fv = -u
%               Fissato x1 = x1_e = theta_e;
%               u_e = -1/2*sin(theta_e) * Fv
x1_e = theta_e;
x2_e = 0;
u_e = -1/2*sin(x1_e) * Fv;
x1 = theta_e;
x2 = 0;
Ae = [ 0                , 1
    a/2*cos(x1_e)*Fv/Id , -beta/Id];

Be = [  0;
        a/Id ];
Ce = [1,0];
De = 0;

%% Calcolo funzione di trasferimento

% G = C * inv(s*eye(2) - A ) * B;
s = tf('s');
ltiSys = ss(Ae,Be,Ce,De);
GG = zpk(ltiSys); % G(s)
zero(GG)
pole(GG)
%pzplot(GG)
%grid on

%% Progettazione del regolatore statico

omega_plot_min = 1e-1;
omega_plot_max = 1e5;

h_GGe = bodeplot(GG,{omega_plot_min,omega_plot_max});
grid on;
title('Diagramma di Bode G(s)');

mu_s = 1;
% Regolatore statico con polo nell'origine per specifica su errore a regime nullo
RR_s = mu_s/s;
GG_e = RR_s * GG; % Calcolo G(s) estesa


%% Mappatura specifiche (realizzata nella funzione create_bode.m)
% figure(2)
% % Specifiche su disturbo di misura n(t)
% omega_n_min = 2e4; % Dato da specifica
% omega_n_MAX = 1e5; % Valore arbitrario per disegnare il rettangolo
% An = 30; % Dato da specifica
% Bnd_n_x = [omega_n_min; omega_n_MAX; omega_n_MAX; omega_n_min];
% Bnd_n_y = [-An*log10(10); -An*log10(10); 100*log10(10); 100*log10(10)];
% patch(Bnd_n_x, Bnd_n_y,'g','FaceAlpha',0.2,'EdgeAlpha',0);
% hold on;
% grid on, zoom on;
%
% % Specifiche sulla sovraelongazione percentuale
% S_perc_spec = 1;
% %xi = calcola_xi(S_perc_spec); % Per calcolarla in modo preciso
% xi = 0.83;
% S_perc = 100*exp(-pi*xi/sqrt(1-xi^2));
% Mf_spec = xi*100;
%
% % Specifiche sul tempo di assestamento
% Ta_spec = 0.4;
% omega_Ta_low = 1e-2; % lower bound just for the plot
% omega_Ta_MAX = 460/(Mf_spec*Ta_spec);   % omega_c >= 460/(Mf*T^*) = 460/(50*0.46) ~ 20
%
% Bnd_Ta_x = [omega_Ta_low; omega_Ta_MAX; omega_Ta_MAX; omega_Ta_low];
% Bnd_Ta_y = [0*log10(10); 0*log10(10); -80*log10(10); -80*log10(10)];
% patch(Bnd_Ta_x, Bnd_Ta_y,'y','FaceAlpha',0.2,'EdgeAlpha',0);
% hold on;
%
% h_GGe = bodeplot(GG_e,{omega_plot_min,omega_plot_max})
% grid on, zoom on;
%
% figure(2)
% omega_Mf_low = 20;
% omega_Mf_up = 200;
% phi_spec = Mf_spec - 180;
%
% Bnd_Mf_x = [omega_Mf_low; omega_Mf_up; omega_Mf_up; omega_Mf_low];
% Bnd_Mf_y = [phi_spec; phi_spec; -270; -270];
% patch(Bnd_Mf_x, Bnd_Mf_y,'r','FaceAlpha',0.2,'EdgeAlpha',0);
% hold on;


create_bode(GG_e,"Ge(j\omega)","Diagramma di Bode G(s)_{estesa}");

%% Progettazione regolatore dinamico

R_1 = 1 + s;    % Inseriamo uno zero vicino l'origine per recuperare 90 gradi di fase
% Avendo un polo nell'origine nel regolatore statico il principio di causalita' e' rispettato
create_bode(GG_e * R_1,"Ge(j\omega)","G(s)_{estesa} con zero");
%% Rete anticipatrice
% Utilizziamo una rete anticipatrice per rientrare nel margine di fase;
% tau1 e a1 sono stati selezionati in modo da avere un aumento di fase
% massimo rispettando le specifiche
% a1 molto piccolo --> maggiore distanza fra zero e polo  --> maggiore
% guadagno di fase (e modulo!!!)

tau1 = 0.2;
%tau2 = 0.05;
a1 = 0.0001;
%a2 = 0.01;
R_ant1=(1+tau1*s)/(1+a1*tau1*s);
%R_ant2=(1+tau2*s)/(1+a2*tau2*s);
%figure()
%bode(R_ant1); grid on;
%title("Rete anticipatrice")
create_bode(GG_e * R_1 * R_ant1 ,"L(j\omega)","Diagramma di Bode di L (senza guadagno)");
% Potendo modificare il guadagno del regolatore e dunque la pulsazione
% di taglio, la prendiamo al picco piu alto di fase per avere un maggiore
% margine di fase
gain=1/abs(evalfr(GG_e * R_1* R_ant1,1i*500));
L = GG_e * R_1*R_ant1 * gain;

create_bode(L,"L(j\omega)","");
figure();
step(L/(1+L),1)
grid on; zoom on;
title("Risposta in retroazione al gradino");
stepinfo(L/(1+L))


%% Simulink
open("SimProgetto");
y_e = x1_e;
omega_n=2e04; % Specifica su disturbo di misura
amplitude_n = 0.03;
W = -theta_e;


R = RR_s * R_1 * R_ant1 * gain;
[n_r,d_r] = tfdata(R);
num_r=n_r{1};
den_r=d_r{1};

G = GG;
[n_g,d_g] = tfdata(G);
num_g=n_g{1};
den_g=d_g{1};

% Condizione iniziale
x0=[theta_e;0];


%% Plot andamendo con condizioni iniziali diverse

if 0    %inserire 1 se si vuole plottare i grafici facendo diverse simulazioni(Richiede molto tempo!)
    figure(99);
    W = -theta_e;
    for j = -2*pi:pi:2*pi  % Variazioni della velocita' angolare
        for i = -pi/2:pi/6:pi/2 % Variazioni dell' angolo iniziali
            
            x0=[i;j];
            fromSim = sim('SimProgetto','SaveOutput','on','OutputSaveName','yout');
            outputs = fromSim.get('yout');
            
            plot(outputs.get('y').Values);
            % xlim([0 0.6])
            
            % ylim([-0.4 0.4])
            grid on; hold on;
        end
    end
    ylabel('y','interpreter','tex');
    title('Risposta del sistema a variazioni delle condizioni iniziali (x_1,x_2)','interpreter','tex');
    %     for iter=1:35
    %         Legend{iter} = num2str(iter);
    %         legend(Legend);
    %     end
    
    
    %% Plot variazioni del riferimento W nell intorno del punto di equilibrio
    figure(101);
    x0=[theta_e;0];
    for i = -pi:pi/6:pi
        W = i;
        fromSim = sim('SimProgetto','SaveOutput','on','OutputSaveName','yout');
        outputs = fromSim.get('yout');
        timeseries = outputs.get('y').Values;
        plot(outputs.get('y').Values);
        %xlim([0 0.6])
        %ylim([-0.5 0.5])
        grid on; hold on;
        
    end
    ylabel('W (+ y_e)','interpreter','tex');
    title('Risposta del sistema a variazioni di \DeltaW','interpreter','tex');
    % for iter=1:12
    %     Legend{iter} = num2str(iter);
    %     legend(Legend);
    % end
end
