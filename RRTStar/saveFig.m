function saveFig(filename, figFormat)
% saveFig(filename, figFormat) saves the current figure.

if strcmp(figFormat, 'png')
    print('-dpng', '-r400', filename);
else
    print('-depsc2', filename);
end

end