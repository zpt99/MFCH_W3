classdef airfoilplotterappv5 < matlab.apps.AppBase

    % Properties that correspond to app components
    properties (Access = public)
        UIFigure                      matlab.ui.Figure
        GridLayout                    matlab.ui.container.GridLayout
        LeftPanel                     matlab.ui.container.Panel
        ReEND                         matlab.ui.control.NumericEditField
        ReynoldsatEndplateEditFieldLabel  matlab.ui.control.Label
        ReMAIN                        matlab.ui.control.NumericEditField
        ReynoldsatMainwingEditFieldLabel  matlab.ui.control.Label
        RADIUS                        matlab.ui.control.NumericEditField
        RadiusofCurvarturemmLabel     matlab.ui.control.Label
        UpdateButton                  matlab.ui.control.Button
        Image                         matlab.ui.control.Image
        PositioningLabel              matlab.ui.control.Label
        cEnd                          matlab.ui.control.NumericEditField
        ChordrelativetomainRWchordmEditFieldLabel  matlab.ui.control.Label
        TEnd                          matlab.ui.control.NumericEditField
        ThicknessratiochordEditField_2Label  matlab.ui.control.Label
        MPosEnd                       matlab.ui.control.NumericEditField
        MaximumcamberpositionchordEditField_2Label  matlab.ui.control.Label
        MCamberEnd                    matlab.ui.control.NumericEditField
        MaximumcamberchordEditField_2Label  matlab.ui.control.Label
        EndplateRWLabel               matlab.ui.control.Label
        Npoints                       matlab.ui.control.NumericEditField
        NumberofpointsEditFieldLabel  matlab.ui.control.Label
        V                             matlab.ui.control.NumericEditField
        VerticaldisplacementmmLabel   matlab.ui.control.Label
        H                             matlab.ui.control.NumericEditField
        HorizontaldistancefromoriginchordEditFieldLabel  matlab.ui.control.Label
        cRW                           matlab.ui.control.NumericEditField
        ChordmLabel                   matlab.ui.control.Label
        TRW                           matlab.ui.control.NumericEditField
        ThicknessratiochordLabel      matlab.ui.control.Label
        MPosRW                        matlab.ui.control.NumericEditField
        MaximumcamberpositionchordLabel  matlab.ui.control.Label
        MainRWLabel                   matlab.ui.control.Label
        MCamberRW                     matlab.ui.control.NumericEditField
        MaximumcamberchordLabel       matlab.ui.control.Label
        RightPanel                    matlab.ui.container.Panel
        SaveDRSButton                 matlab.ui.control.Button
        SAVEDTEXT                     matlab.ui.control.TextArea
        plotDRS                       matlab.ui.control.UIAxes
    end

    % Properties that correspond to apps with auto-reflow
    properties (Access = private)
        onePanelWidth = 576;
    end

    
    methods (Access = private)
        function [R]=curvature(app)
            m=app.MCamberRW.Value; 
            p=app.MPosRW.Value;  
            L=app.Npoints.Value; 
            
            M = m/100;              % max camber
            P = p/100;              % max camber position
            c=app.cRW.Value;        % main rear wing chord 
            
            
            %% initialize
            beta=linspace(0,pi,L);
            CAMBERx=zeros(1,L);
            for j=1:L
                CAMBERx(j)=c*(1-(cos(beta(j))))/2;
            end
            
            %% plotter            
            for i=1:L
                if CAMBERx(i)<=P*c
                    CAMBERy(i)=(M*CAMBERx(i)/P^2)*(2*P-(CAMBERx(i)/c));
                else
                    CAMBERy(i)=(M*(c-CAMBERx(i))/(1-P)^2)*(1+CAMBERx(i)/c-2*P);
                end
            end
            CAMBERy=-CAMBERy;
            for i=1:L-1
                if (CAMBERx(i)<= P*c && CAMBERx(i+1) >= P*c)
                    aux1=i;
                end
            end
            if(m>=30)
                N=round(0.15*L);
                last=round(N);
            else
                N=round(0.3*L);
                if aux1-N<=0
                    N=aux1-10;
                elseif aux1+N>=L
                    N=L-aux1-10;
                end
                last=round(N/2);
            end

            Coord=[CAMBERx(aux1-N:aux1+last)' CAMBERy(aux1-N:aux1+last)'];
            circlefun = @(b,Coord) (Coord(:,1).^2 + Coord(:,2).^2 + b(1)*Coord(:,1) + b(2)*Coord(:,2) + b(3));
            y = zeros(length(Coord(:,1)),1);
            beta0 = [0 0 400];
            mdl = fitnlm(Coord,y,circlefun, beta0);
            B = mdl.Coefficients.Estimate;
            Xm = -B(1)/2;
            Ym = -B(2)/2;
            R = sqrt((Xm^2 + Ym^2) - B(3));
            A = atan2(Coord(:,2)-Ym, Coord(:,1)-Xm);
            Ycir = R*sin(A) + Ym;
            Xcir = R*cos(A) + Xm;
            figure(2)
            % plot(Coord(:,1), Coord(:,2), 'p')
            % hold on
            p = nsidedpoly(1000, 'Center', [Xm Ym], 'Radius', R);
            plot(p, 'FaceColor', 'r')
            hold on
            plot(CAMBERx,CAMBERy,'LineWidth',3)
            plot(Coord(:,1),Coord(:,2),'green','LineWidth',3)
            hold off
            grid
            axis equal 
            R=R*1000;
        end
        
        function saveAirfoil(app,XRW,YRW,XEnd,YEnd,XFLR_X,XFLR_Y,M,M2,P,P2,T,T2,L,X,Y,CAMBER,CAMBER2)
                %% SAVE
                %% save data points
                name='DataPoints';
                mkdir(name)

                Z=zeros(1,2*L);
                AIRFOILXFLR=[X;Y];
                AIRFOILXFLRend=[XFLR_X;XFLR_Y];
                AIRFOIL=[XRW;YRW;Z]';
                AIRFOIL2=[XEnd;YEnd;Z]';

                %% SOLIDWORKS
                writematrix(AIRFOIL,[name,'/DRSmain_M',num2str(M*100),'Pos',num2str(P*100),'T',num2str(T*100),'.csv']);
                writematrix(AIRFOIL2,[name,'/DRSendplate_M2_',num2str(M2*100),'Pos2_',num2str(P2*100),'T2_',num2str(T2*100),'.csv']);

                %% XFLR5
                fid = fopen([name,'/DRSmain_M',num2str(M*100),'Pos',num2str(P*100),'T',num2str(T*100),'.dat'], 'w');
                fprintf(fid,'%s\r\n','MainWing');
                fprintf(fid,'\r\n');
                fprintf(fid,'%10.6f %10.6f\r\n',CAMBER);
                fprintf(fid,'\r\n');
                fprintf(fid,'%10.6f %10.6f\r\n',AIRFOILXFLR);
                fclose(fid);
                fid2 = fopen([name,'/DRSendplate_M2_',num2str(M2*100),'Pos2_',num2str(P2*100),'T2_',num2str(T2*100),'.dat'], 'w');
                fprintf(fid2,'%s\r\n','EndPlate');
                fprintf(fid,'\r\n');
                fprintf(fid,'%10.6f %10.6f\r\n',CAMBER2);
                fprintf(fid,'\r\n');
                fprintf(fid2,'%10.6f %10.6f\r\n',AIRFOILXFLRend);
                fclose(fid2);
        end
        
        function [XRW,YRW,xc,yc,XEnd,YEnd,XFLR_X,XFLR_Y,xc2,yc2,M,M2,P,P2,T,T2,L,X,Y,CAMBER,CAMBER2]=doairfoils(app)
                 
          
            %% inputs

            m=app.MCamberRW.Value; 
            p=app.MPosRW.Value; 
            t=app.TRW.Value; 
            m2=app.MCamberEnd.Value; 
            p2=app.MPosEnd.Value; 
            t2=app.TEnd.Value; 
            L=app.Npoints.Value; 
            
            M = m/100; M2=m2/100;   % max camber
            P = p/100; P2=p2/100;   % max camber position 
            T = t/100; T2=t2/100;   % thickness ratio
            c=app.cRW.Value;        % main rear wing chord 
            d=app.cEnd.Value;       % percentage of main rear wing chord
            c2=c*d;                 % endplate chord = percentage of main rear wing chord
            
            
            %% Positioning
            h=app.H.Value*c;          % distance between origin and LE of endplate
            v=app.V.Value/1000;       % vertical distance between LE and longitunidal axis
            
            %% Main Rear Wing
            %% initialize
            beta=linspace(0,pi,L);
            theta=zeros(1,L);
            xc=zeros(1,L);
            for j=1:L
                xc(j)=c*(1-(cos(beta(j))))/2;
            end
            xc=sort(xc,'descend');
            
            %% plotter
            % thickness distribution constants
            a0=0.2969; a1=-0.126; a2=-0.3516; a3=0.2843; a4=-0.1015;
            
            for i = 1:L
                if xc(i)<=P*c
                    yc(i)=M*xc(i)/P^2*(2*P-(xc(i)/c));
                    dyc(i)=2*M/P^2*(P-xc(i)/c);
                else
                    yc(i)=(M*(c-xc(i))/(1-P)^2)*(1+xc(i)/c-2*P);
                    dyc(i)=(2*M/((1-P)^2))*(P-xc(i)/c);
                end
            end
            xC=xc/c;
            yt=(T*c/0.2)*(a0*(xC).^0.5+a1*xC+a2*(xC).^2+a3*(xC).^3+a4*(xC).^4);
            theta=atan(dyc);
            
            xu=xc-yt.*sin(theta);
            xl=xc+yt.*sin(theta);
            
            yu=yc+yt.*cos(theta);
            yl=yc-yt.*cos(theta);

            
            %MAIN
            xL=zeros(1,L);
            yL=zeros(1,L);
            for aux=0:L-1
                xL(aux+1)=xu(L-aux);
                xlo(aux+1)=xl(L-aux);
                yL(aux+1)=-(yu(L-aux));
                ylo(aux+1)=yl(L-aux);
            end
            xU=xl;
            xup=xu;
            yU=-(yl);
            yup=yu;
            yc=-yc;
            
            X=[xup,xlo];
            Y=[yup,ylo];
         
            CAMBER=[xc;-yc];
            
            XRW= [xU, xL];
            YRW= [yU, yL];
            
            %% ENDPLATE
            %% initialize
            xc2=zeros(1,L);
            theta2=zeros(1,L);
            for j=1:L
                xc2(j)=c2*((1-cos(beta(j)))/2);
            end
            xc2=sort(xc2,'descend');
            
            %% plotter
            % thickness distribution constants
            a0=0.2969; a1=-0.126; a2=-0.3516; a3=0.2843; a4=-0.1015;
            for i = 1:L
                
                if xc2(i)<=(P2*c2)
                    yc2(i)=(M2*xc2(i)/(P2)^2)*(2*(P2)-xc2(i)/c2);
                    dyc2(i)=2*M2/((P2)^2)*(P2-xc2(i)/c2);
                else
                    yc2(i)=(M2*(c2-xc2(i))/(1-(P2))^2)*(1+xc2(i)/c2-2*P2);
                    dyc2(i)=(2*M2/((1-P2)^2))*(P2-xc2(i)/c2);
                end
            end
            xC2=xc2/c2;
            yt2=(T2*c2/0.2)*(a0*xC2.^0.5+a1*xC2+a2*xC2.^2+a3*xC2.^3+a4*xC2.^4);
            theta2=atan(dyc2);
            
            xu2=xc2-yt2.*sin(theta2);
            xl2=xc2+yt2.*sin(theta2);
            
            yu2=yc2+yt2.*cos(theta2);
            yl2=yc2-yt2.*cos(theta2);

            
            CAMBER2=[xc2;-yc2];
            %% organize
            xL2=zeros(1,L);
            yL2=zeros(1,L);
            for aux=0:L-1
                xL2(aux+1)=xu2(L-aux)+h;
                xl2flr(aux+1)=xl2(L-aux);
                yL2(aux+1)=-(yu2(L-aux))+v;
                yl2flr(aux+1)=(yl2(L-aux));
            end
            
            xU2=xl2+h;
            xu2flr=xu2;
            yU2=-(yl2)+v;
            yu2flr=(yu2);
            xc2=xc2+h;
            yc2=-(yc2)+v;

            XEnd=[xU2,xL2];
            YEnd=[yU2,yL2];

            XFLR_X=[xu2flr,xl2flr];
            XFLR_Y=[yu2flr,yl2flr];
        end
    end
    

    % Callbacks that handle component events
    methods (Access = private)

        % Changes arrangement of the app based on UIFigure width
        function updateAppLayout(app, event)
            currentFigureWidth = app.UIFigure.Position(3);
            if(currentFigureWidth <= app.onePanelWidth)
                % Change to a 2x1 grid
                app.GridLayout.RowHeight = {717, 717};
                app.GridLayout.ColumnWidth = {'1x'};
                app.RightPanel.Layout.Row = 2;
                app.RightPanel.Layout.Column = 1;
            else
                % Change to a 1x2 grid
                app.GridLayout.RowHeight = {'1x'};
                app.GridLayout.ColumnWidth = {576, '1x'};
                app.RightPanel.Layout.Row = 1;
                app.RightPanel.Layout.Column = 2;
            end
        end

        % Button pushed function: UpdateButton
        function UpdateButtonPushed(app, event)
               
            c=app.cRW.Value;        % main rear wing chord 
            d=app.cEnd.Value;       % percentage of main rear wing chord
            c2=c*d;                 % endplate chord = percentage of main rear wing chord
              
            V=300/3.6;
            nu=1.5111E-5;
            app.ReMAIN.Value=V*c/nu;
            app.ReEND.Value=V*c2/nu;
            
            [XRW,YRW,xc,yc,XEnd,YEnd,~,~,xc2,yc2,~,~,~,~,~,~,~,~,~,~,~]=doairfoils(app);
            plot(app.plotDRS, XRW,YRW,'blue',xc,yc,'red--',XEnd,YEnd,'blue',xc2,yc2,'red--') ;
            app.plotDRS.DataAspectRatio = [1 1 1];
            app.RADIUS.Value=curvature(app);

        end

        % Button pushed function: SaveDRSButton
        function SaveDRSButtonPushed(app, event)
            [XRW,YRW,~,~,XEnd,YEnd,XFLR_X,XFLR_Y,~,~,M,M2,P,P2,T,T2,L,X,Y,CAMBER,CAMBER2]=doairfoils(app);
            saveAirfoil(app,XRW,YRW,XEnd,YEnd,XFLR_X,XFLR_Y,M,M2,P,P2,T,T2,L,X,Y,CAMBER,CAMBER2)
            app.SAVEDTEXT.Value="DRS saved successfully.";
        end
    end

    % Component initialization
    methods (Access = private)

        % Create UIFigure and components
        function createComponents(app)

            % Create UIFigure and hide until all components are created
            app.UIFigure = uifigure('Visible', 'off');
            app.UIFigure.AutoResizeChildren = 'off';
            app.UIFigure.Position = [100 100 1210 717];
            app.UIFigure.Name = 'MATLAB App';
            app.UIFigure.SizeChangedFcn = createCallbackFcn(app, @updateAppLayout, true);

            % Create GridLayout
            app.GridLayout = uigridlayout(app.UIFigure);
            app.GridLayout.ColumnWidth = {576, '1x'};
            app.GridLayout.RowHeight = {'1x'};
            app.GridLayout.ColumnSpacing = 0;
            app.GridLayout.RowSpacing = 0;
            app.GridLayout.Padding = [0 0 0 0];
            app.GridLayout.Scrollable = 'on';

            % Create LeftPanel
            app.LeftPanel = uipanel(app.GridLayout);
            app.LeftPanel.TitlePosition = 'centertop';
            app.LeftPanel.Title = 'Airfoil Selection';
            app.LeftPanel.Layout.Row = 1;
            app.LeftPanel.Layout.Column = 1;
            app.LeftPanel.FontWeight = 'bold';

            % Create MaximumcamberchordLabel
            app.MaximumcamberchordLabel = uilabel(app.LeftPanel);
            app.MaximumcamberchordLabel.HorizontalAlignment = 'right';
            app.MaximumcamberchordLabel.WordWrap = 'on';
            app.MaximumcamberchordLabel.Position = [44 603 117 43];
            app.MaximumcamberchordLabel.Text = 'Maximum camber (%chord):';

            % Create MCamberRW
            app.MCamberRW = uieditfield(app.LeftPanel, 'numeric');
            app.MCamberRW.Position = [195 613 34 22];
            app.MCamberRW.Value = 5;

            % Create MainRWLabel
            app.MainRWLabel = uilabel(app.LeftPanel);
            app.MainRWLabel.HorizontalAlignment = 'center';
            app.MainRWLabel.FontWeight = 'bold';
            app.MainRWLabel.Position = [91 658 125 22];
            app.MainRWLabel.Text = 'Main RW';

            % Create MaximumcamberpositionchordLabel
            app.MaximumcamberpositionchordLabel = uilabel(app.LeftPanel);
            app.MaximumcamberpositionchordLabel.HorizontalAlignment = 'right';
            app.MaximumcamberpositionchordLabel.WordWrap = 'on';
            app.MaximumcamberpositionchordLabel.Position = [36 559 121 41];
            app.MaximumcamberpositionchordLabel.Text = 'Maximum camber position (%chord):';

            % Create MPosRW
            app.MPosRW = uieditfield(app.LeftPanel, 'numeric');
            app.MPosRW.Position = [195 568 34 22];
            app.MPosRW.Value = 40;

            % Create ThicknessratiochordLabel
            app.ThicknessratiochordLabel = uilabel(app.LeftPanel);
            app.ThicknessratiochordLabel.HorizontalAlignment = 'right';
            app.ThicknessratiochordLabel.WordWrap = 'on';
            app.ThicknessratiochordLabel.Position = [39 516 119 30];
            app.ThicknessratiochordLabel.Text = 'Thickness ratio (%chord):';

            % Create TRW
            app.TRW = uieditfield(app.LeftPanel, 'numeric');
            app.TRW.Position = [195 520 34 22];
            app.TRW.Value = 12;

            % Create ChordmLabel
            app.ChordmLabel = uilabel(app.LeftPanel);
            app.ChordmLabel.HorizontalAlignment = 'right';
            app.ChordmLabel.Position = [95 476 63 22];
            app.ChordmLabel.Text = 'Chord (m):';

            % Create cRW
            app.cRW = uieditfield(app.LeftPanel, 'numeric');
            app.cRW.Position = [195 476 35 22];
            app.cRW.Value = 0.5;

            % Create HorizontaldistancefromoriginchordEditFieldLabel
            app.HorizontaldistancefromoriginchordEditFieldLabel = uilabel(app.LeftPanel);
            app.HorizontaldistancefromoriginchordEditFieldLabel.HorizontalAlignment = 'right';
            app.HorizontaldistancefromoriginchordEditFieldLabel.WordWrap = 'on';
            app.HorizontaldistancefromoriginchordEditFieldLabel.Position = [35 366 120 42];
            app.HorizontaldistancefromoriginchordEditFieldLabel.Text = 'Horizontal distance from origin (%chord):';

            % Create H
            app.H = uieditfield(app.LeftPanel, 'numeric');
            app.H.Position = [195 378 34 22];
            app.H.Value = 0.7;

            % Create VerticaldisplacementmmLabel
            app.VerticaldisplacementmmLabel = uilabel(app.LeftPanel);
            app.VerticaldisplacementmmLabel.HorizontalAlignment = 'right';
            app.VerticaldisplacementmmLabel.WordWrap = 'on';
            app.VerticaldisplacementmmLabel.Position = [35 324 120 28];
            app.VerticaldisplacementmmLabel.Text = 'Vertical displacement (mm):';

            % Create V
            app.V = uieditfield(app.LeftPanel, 'numeric');
            app.V.Position = [195 330 34 22];
            app.V.Value = 100;

            % Create NumberofpointsEditFieldLabel
            app.NumberofpointsEditFieldLabel = uilabel(app.LeftPanel);
            app.NumberofpointsEditFieldLabel.HorizontalAlignment = 'right';
            app.NumberofpointsEditFieldLabel.Position = [308 357 100 22];
            app.NumberofpointsEditFieldLabel.Text = 'Number of points:';

            % Create Npoints
            app.Npoints = uieditfield(app.LeftPanel, 'numeric');
            app.Npoints.Position = [446 357 34 22];
            app.Npoints.Value = 100;

            % Create EndplateRWLabel
            app.EndplateRWLabel = uilabel(app.LeftPanel);
            app.EndplateRWLabel.HorizontalAlignment = 'center';
            app.EndplateRWLabel.FontWeight = 'bold';
            app.EndplateRWLabel.Position = [331 658 125 22];
            app.EndplateRWLabel.Text = 'Endplate RW';

            % Create MaximumcamberchordEditField_2Label
            app.MaximumcamberchordEditField_2Label = uilabel(app.LeftPanel);
            app.MaximumcamberchordEditField_2Label.HorizontalAlignment = 'right';
            app.MaximumcamberchordEditField_2Label.WordWrap = 'on';
            app.MaximumcamberchordEditField_2Label.Position = [293 603 117 43];
            app.MaximumcamberchordEditField_2Label.Text = 'Maximum camber (%chord):';

            % Create MCamberEnd
            app.MCamberEnd = uieditfield(app.LeftPanel, 'numeric');
            app.MCamberEnd.Position = [481 613 33 22];
            app.MCamberEnd.Value = 5;

            % Create MaximumcamberpositionchordEditField_2Label
            app.MaximumcamberpositionchordEditField_2Label = uilabel(app.LeftPanel);
            app.MaximumcamberpositionchordEditField_2Label.HorizontalAlignment = 'right';
            app.MaximumcamberpositionchordEditField_2Label.WordWrap = 'on';
            app.MaximumcamberpositionchordEditField_2Label.Position = [290 561 121 41];
            app.MaximumcamberpositionchordEditField_2Label.Text = 'Maximum camber position (%chord):';

            % Create MPosEnd
            app.MPosEnd = uieditfield(app.LeftPanel, 'numeric');
            app.MPosEnd.Position = [481 570 34 22];
            app.MPosEnd.Value = 40;

            % Create ThicknessratiochordEditField_2Label
            app.ThicknessratiochordEditField_2Label = uilabel(app.LeftPanel);
            app.ThicknessratiochordEditField_2Label.HorizontalAlignment = 'right';
            app.ThicknessratiochordEditField_2Label.WordWrap = 'on';
            app.ThicknessratiochordEditField_2Label.Position = [293 518 119 30];
            app.ThicknessratiochordEditField_2Label.Text = 'Thickness ratio (%chord):';

            % Create TEnd
            app.TEnd = uieditfield(app.LeftPanel, 'numeric');
            app.TEnd.Position = [481 522 34 22];
            app.TEnd.Value = 10;

            % Create ChordrelativetomainRWchordmEditFieldLabel
            app.ChordrelativetomainRWchordmEditFieldLabel = uilabel(app.LeftPanel);
            app.ChordrelativetomainRWchordmEditFieldLabel.HorizontalAlignment = 'right';
            app.ChordrelativetomainRWchordmEditFieldLabel.WordWrap = 'on';
            app.ChordrelativetomainRWchordmEditFieldLabel.Position = [299 473 110 33];
            app.ChordrelativetomainRWchordmEditFieldLabel.Text = 'Chord relative to main RW chord (m):';

            % Create cEnd
            app.cEnd = uieditfield(app.LeftPanel, 'numeric');
            app.cEnd.Position = [481 476 33 22];
            app.cEnd.Value = 0.5;

            % Create PositioningLabel
            app.PositioningLabel = uilabel(app.LeftPanel);
            app.PositioningLabel.HorizontalAlignment = 'center';
            app.PositioningLabel.Position = [91 411 125 22];
            app.PositioningLabel.Text = 'Positioning';

            % Create Image
            app.Image = uiimage(app.LeftPanel);
            app.Image.Position = [28 13 327 294];
            app.Image.ImageSource = 'TwoDim.jpg';

            % Create UpdateButton
            app.UpdateButton = uibutton(app.LeftPanel, 'push');
            app.UpdateButton.ButtonPushedFcn = createCallbackFcn(app, @UpdateButtonPushed, true);
            app.UpdateButton.Position = [416 227 100 22];
            app.UpdateButton.Text = {'Update'; ''};

            % Create RadiusofCurvarturemmLabel
            app.RadiusofCurvarturemmLabel = uilabel(app.LeftPanel);
            app.RadiusofCurvarturemmLabel.HorizontalAlignment = 'right';
            app.RadiusofCurvarturemmLabel.WordWrap = 'on';
            app.RadiusofCurvarturemmLabel.Position = [364 166 89 29];
            app.RadiusofCurvarturemmLabel.Text = 'Radius of Curvarture (mm)';

            % Create RADIUS
            app.RADIUS = uieditfield(app.LeftPanel, 'numeric');
            app.RADIUS.ValueDisplayFormat = '%11g';
            app.RADIUS.Editable = 'off';
            app.RADIUS.Position = [475 169 88 22];

            % Create ReynoldsatMainwingEditFieldLabel
            app.ReynoldsatMainwingEditFieldLabel = uilabel(app.LeftPanel);
            app.ReynoldsatMainwingEditFieldLabel.HorizontalAlignment = 'right';
            app.ReynoldsatMainwingEditFieldLabel.WordWrap = 'on';
            app.ReynoldsatMainwingEditFieldLabel.Position = [345 121 89 29];
            app.ReynoldsatMainwingEditFieldLabel.Text = 'Reynolds at Main wing';

            % Create ReMAIN
            app.ReMAIN = uieditfield(app.LeftPanel, 'numeric');
            app.ReMAIN.ValueDisplayFormat = '%11g';
            app.ReMAIN.Editable = 'off';
            app.ReMAIN.Position = [456 124 107 22];

            % Create ReynoldsatEndplateEditFieldLabel
            app.ReynoldsatEndplateEditFieldLabel = uilabel(app.LeftPanel);
            app.ReynoldsatEndplateEditFieldLabel.HorizontalAlignment = 'right';
            app.ReynoldsatEndplateEditFieldLabel.WordWrap = 'on';
            app.ReynoldsatEndplateEditFieldLabel.Position = [345 74 89 29];
            app.ReynoldsatEndplateEditFieldLabel.Text = 'Reynolds at Endplate';

            % Create ReEND
            app.ReEND = uieditfield(app.LeftPanel, 'numeric');
            app.ReEND.ValueDisplayFormat = '%11g';
            app.ReEND.Editable = 'off';
            app.ReEND.Position = [456 77 107 22];

            % Create RightPanel
            app.RightPanel = uipanel(app.GridLayout);
            app.RightPanel.TitlePosition = 'centertop';
            app.RightPanel.Title = 'DRS System';
            app.RightPanel.Layout.Row = 1;
            app.RightPanel.Layout.Column = 2;
            app.RightPanel.FontWeight = 'bold';

            % Create plotDRS
            app.plotDRS = uiaxes(app.RightPanel);
            title(app.plotDRS, 'DRS section view')
            xlabel(app.plotDRS, '[m]')
            ylabel(app.plotDRS, '[m]')
            zlabel(app.plotDRS, 'Z')
            app.plotDRS.ClippingStyle = 'rectangle';
            app.plotDRS.XGrid = 'on';
            app.plotDRS.YGrid = 'on';
            app.plotDRS.Box = 'on';
            app.plotDRS.Position = [21 111 592 493];

            % Create SAVEDTEXT
            app.SAVEDTEXT = uitextarea(app.RightPanel);
            app.SAVEDTEXT.HorizontalAlignment = 'center';
            app.SAVEDTEXT.Position = [274 21 240 38];

            % Create SaveDRSButton
            app.SaveDRSButton = uibutton(app.RightPanel, 'push');
            app.SaveDRSButton.ButtonPushedFcn = createCallbackFcn(app, @SaveDRSButtonPushed, true);
            app.SaveDRSButton.Position = [128 29 101 23];
            app.SaveDRSButton.Text = 'Save DRS';

            % Show the figure after all components are created
            app.UIFigure.Visible = 'on';
        end
    end

    % App creation and deletion
    methods (Access = public)

        % Construct app
        function app = airfoilplotterappv5

            % Create UIFigure and components
            createComponents(app)

            % Register the app with App Designer
            registerApp(app, app.UIFigure)

            if nargout == 0
                clear app
            end
        end

        % Code that executes before app deletion
        function delete(app)

            % Delete UIFigure when app is deleted
            delete(app.UIFigure)
        end
    end
end