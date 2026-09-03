fig = openfig('E1_sa.fig');

set(fig, 'WindowStyle','normal');
set(fig, 'Units','centimeters');

column_width = 8.2;
height = 12;

set(fig, 'Position',[2 2 column_width height]);
set(gca, 'FontSize', 8);
set(fig, 'Color','w');

set(fig, 'PaperUnits','centimeters');
set(fig, 'PaperPosition',[0 0 column_width height]);
set(fig, 'PaperSize',[column_width height]);

print(fig, 'sa', '-dpdf', '-painters');
close(fig);