function distances = pdist2B(Point, PMatrix)
    %pdist2B - this function provides an alternate to using the
    % MATLAB Statistics and Machine Learning Toolbox.
    if exist('pdist2', 'file') ~= 0
        % Toolbox found. Use that instead
        distances = pdist2(Point, PMatrix);
    else
        % No toolbox. Calculate distances.

        distances = NaN(1, size(PMatrix, 1));

        for i = 1:size(PMatrix, 1)
            distances(i) = norm(Point - PMatrix(i, :));
        end

    end

end
