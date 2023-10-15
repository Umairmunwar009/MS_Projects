function [fitresult,ft_path] = createFit(xxx, yyy, curve_para)
%CREATEFIT(XXX,YYY)
%  Create a fit.
%
%  Data for 'curve_fit' fit:
%      X Input : xxx
%      Y Output: yyy
%  Output:
%      fitresult : a fit object representing the fit.
%      gof : structure with goodness-of fit info.
%
%  See also FIT, CFIT, SFIT.

%  Auto-generated by MATLAB on 08-Apr-2020 15:24:53

curve_para=curve_para*1e-6;
%% Fit: 'curve_fit'.
[xData, yData] = prepareCurveData( xxx, yyy );

% Set up fittype and options.
ft = fittype( 'smoothingspline' );

opts = fitoptions( 'Method', 'SmoothingSpline' );
opts.SmoothingParam = curve_para;      %1.26315781118076e-05

% Fit model to data.
[fitresult, gof] = fit( xData, yData, ft, opts );

fitresult_p = polyfit(xxx,yyy,8);
ft_path = polyval(fitresult_p,xxx);

% Plot fit with data.
% figure(63) %'Name', 'curve_fit' );
% plot( fitresult, xData, yData);hold on;
% plot( xxx,ft_path,'-g' );hold on;
% plot( xData, yData,'oy' );

% legend( 'Green Polyfit', 'Red smooth curve_fit', 'Blue Original' );
% Label axes
% xlabel xxx
% ylabel yyy
% grid on


