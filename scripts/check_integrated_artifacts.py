"""Verify a finished recording against physical-step logs and decode the complete H.264."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess


def main():
    parser=argparse.ArgumentParser();parser.add_argument('directory',type=Path);args=parser.parse_args()
    directory=args.directory
    summary=json.loads((directory/'summary.json').read_text(encoding='utf-8'))
    video=json.loads((directory/'video.json').read_text(encoding='utf-8'))
    rows=[json.loads(line) for line in (directory/'events.jsonl').read_text(encoding='utf-8').splitlines()]
    samples={round(row['simulated_time_s'],6):row for row in rows if row['event']=='motion_sample'}
    frames=video['frames'];matched=0
    for frame in frames:
        if frame['time_s']==0:continue
        row=samples[frame['time_s']]
        for axis in ('x_m','y_m','yaw_rad'):
            assert abs(row['pose'][axis]-frame['pose'][axis])<1e-8,(frame['time_s'],axis)
        matched+=1
    assert all(0<b['time_s']-a['time_s']<=.066668 for a,b in zip(frames,frames[1:]))
    assert summary['termination']=='success' and summary['source_unchanged']
    assert video['fps']==15
    assert all(summary['evaluation'][key] for key in ('w3_valid','near_evidence','stationary_evidence','distinct_times','terminal_stopped'))
    import imageio_ffmpeg
    ffmpeg=imageio_ffmpeg.get_ffmpeg_exe()
    result=subprocess.run([ffmpeg,'-hide_banner','-i',str(directory/video['file']),'-f','null','-'],
                          stdout=subprocess.PIPE,stderr=subprocess.PIPE,encoding='utf-8',timeout=180)
    assert result.returncode==0,result.stderr[-2000:]
    decoded=int(re.findall(r'frame=\s*(\d+)',result.stderr)[-1])
    assert decoded==len(frames),(decoded,len(frames))
    for time in (5.,summary['simulated_time_s']*.5,max(0.,summary['simulated_time_s']-.15)):
        picture=directory/('check-frame-%.1f.png'%time)
        subprocess.run([ffmpeg,'-v','error','-ss',str(time),'-i',str(directory/video['file']),
                        '-frames:v','1','-y',str(picture)],check=True,timeout=30)
    html=(directory/'index.html').read_text(encoding='utf-8')
    assert '__PAYLOAD__' not in html and 'src="integrated.mp4"' in html
    assert all((directory/name).exists() for name in ('start.png','final.png','evaluation-only.json','manifest.json'))
    report=dict(passed=True,decoded_frames=decoded,fps=15,physical_frames_matched=matched,
                video_max_gap_s=video['max_gap_s'],termination=summary['termination'],
                collisions=summary['collisions'],llm_calls=summary['llm_calls'],llm_fallbacks=summary['llm_fallbacks'],
                source_fingerprint=summary['source_fingerprint'],
                video_sha256=hashlib.sha256((directory/video['file']).read_bytes()).hexdigest())
    (directory/'artifact-verification.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
