% Controllo dell'assetto di un Drone Planare
% Traccia 1c
% Gruppo U
% 30/11/2020
% Funzione a cura di Antonio De Luca

function [Mag,phase,omega] = create_bode(TF,nameTF,titolo)
    % Crea un diagramma di bode a partire da una funzione di trasferimento
    %   TF: funzione di trasferimento
    %   nameTF: nome della funzione di trasferimento (da mostrare nella legenda)
    %   titolo: titolo del grafico/ ""(stringa vuota) per i margine di
    %   Bisogna inserire manualmente tutte le specifiche  
    
    
    %%%%% SPECIFICHE %%%%%
    
    % Range di valori da mostrare
    omega_plot_min = 1e0;
    omega_plot_max = 1e5;
    
    % Specifiche su disturbo di misura n(t)
    omega_n_min = 2e4; % Dato da specifica
    omega_n_MAX = omega_plot_max; % Valore arbitrario per disegnare il rettangolo
    An = 30; % Abbattimento del segnale di disturbo (Dato da specifica)
    An_db = 20*log10(An);
    
    % Specifiche sulla sovraelongazione percentuale   
    %S_perc_spec = 1;
    %xi= sqrt(log(S_perc_spec/100)^2/(pi^2+log(S_perc_spec/100)^2)); % Per calcolarla
    %S_perc = 100*exp(-pi*xi/sqrt(1-xi^2));    
    
    xi = 0.83;
    Mf_spec = xi*100;
    
    % Specifiche sul tempo di assestamento
    Ta_spec = 0.4;
    omega_Ta_low = omega_plot_min; % Valore arbitrario per il rettangolo
    omega_Ta_MAX = 460/(Mf_spec*Ta_spec);   % omega_c >= 460/(Mf*T^*)
    
    % Specifiche sul margine di fase
    omega_Mf_low = omega_Ta_MAX;
    omega_Mf_up = omega_n_min;  
    phi_spec = Mf_spec - 180;
    
    %%%%% FINE  SPECIFICHE %%%%%
    
    [Mag,phase,omega]=bode(TF,{omega_plot_min,omega_plot_max},'k');
    figure()
 
    % Disturbo di misura n(t) (verde)
    Bnd_n_x = [omega_n_min; omega_n_MAX; omega_n_MAX; omega_n_min];
    Bnd_n_y = [-An_db; -An_db; 100; 100];
    patch(Bnd_n_x, Bnd_n_y,'g','FaceAlpha',0.4,'EdgeAlpha',0);
    hold on;
    
    
    
   
    % Tempo di assestamento  (giallo)
    Bnd_Ta_x = [omega_Ta_low; omega_Ta_MAX; omega_Ta_MAX; omega_Ta_low];
    Bnd_Ta_y = [0; 0; -80; -80];
    patch(Bnd_Ta_x, Bnd_Ta_y,'y','FaceAlpha',0.4,'EdgeAlpha',0);
    legend(["A_n";"\omega_{cMin}"]);
    hold on;
    
    
    margin(Mag,phase,omega);grid on; % Disegna il diagramma di bode
    

    % Margine di fase   (rosso)
    Bnd_Mf_x = [omega_Mf_low; omega_Mf_up; omega_Mf_up; omega_Mf_low];
    Bnd_Mf_y = [phi_spec; phi_spec; -270; -270];
    patch(Bnd_Mf_x, Bnd_Mf_y,'r','FaceAlpha',0.4,'EdgeAlpha',0);
    legend([nameTF;"M_f"]);
    hold off;
    
    %Se si inserisce una stringa vuota come titolo usa quello di default
    if titolo ~= ""  
    title(titolo);
    end
    
    % Nota: I diagrammi di Bode di MATLAB sono composti da due figure
    % separate e restituisce il controllo solo dell'ultima figura. Quindi
    % bisogna prima disegnare i rettangoli nel modulo, poi tracciare il
    % diagramma ed infine tracciare il/i rettangolo/i nella fase.
    % Stessa cosa vale per la legenda.
end

