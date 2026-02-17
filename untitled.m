% 현재 열려있는 프로젝트 확인 또는 로드
try
    proj = currentProject;
catch
    % 프로젝트 경로 지정 필요 (.prj 파일 또는 폴더)
    proj = openProject("C:\path\to\SOFC_DCBST_italy");
end