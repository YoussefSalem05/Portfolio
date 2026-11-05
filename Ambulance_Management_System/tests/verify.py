"""Black-box timing, dispatch, cancellation, recovery, and validation checks.
Usage: python tests/verify.py path/to/AmbulanceSystem.exe
No external Python packages are needed.
"""
from pathlib import Path
import subprocess
import sys
import tempfile

ROOT=Path(__file__).resolve().parents[1]
EXE=Path(sys.argv[1]).resolve()
SAMPLES=ROOT/'samples'

def scenario(hospitals, requests, cancels=(), rate=0, speeds=(10,10), checkups=(2,2), matrix=None):
    n=len(hospitals)
    matrix=matrix or [[0 if i==j else 20 for j in range(n)] for i in range(n)]
    lines=[str(n),f'{speeds[0]} {speeds[1]}',f'{checkups[0]} {checkups[1]}',str(rate)]
    lines+=[' '.join(map(str,row)) for row in matrix+hospitals]
    return '\n'.join(lines+[str(len(requests))]+requests+[str(len(cancels))]+list(cancels))+'\n'

cases={
 '01_priority': (scenario([[1,1]],['NP 1 1 1 10','SP 1 2 1 10','EP 1 3 1 10 1','EP 1 4 1 10 9']),
                 [(3,4,1,1),(3,3,1,1),(5,2,1,3),(5,1,1,3)],'Served: 4'),
 '02_cancellations': (scenario([[0,1]],['NP 1 1 1 50','NP 1 2 1 10','NP 1 3 1 10','NP 10 4 1 10'],
                               ['2 2 1','3 1 1','7 3 1','11 4 1']),
                      [(7,3,1,5),(12,4,10,1)],'Cancelled: 2'),
 '03_transfer': (scenario([[0,1],[0,1],[0,1]],['EP 1 1 1 20 9','EP 1 2 1 20 5','EP 1 3 1 10 2'],
                         matrix=[[0,40,10],[40,0,20],[10,20,0]]),
                 [(5,1,1,2),(7,2,1,3),(11,3,1,5)],'EP not served by home hospital: 66.67%'),
 '04_rounding': (scenario([[1,1]],['NP 1 1 1 11','SP 1 2 1 0','NP 2 3 1 0','SP 2 4 1 21']),
                 [(1,2,1,0),(5,1,1,2),(5,3,2,3),(8,4,2,3)],'Served: 4'),
 '05_failures': (scenario([[1,1]],['NP 1 1 1 80','SP 1 2 1 70','EP 2 3 1 20 5','NP 3 4 1 20'],rate=.35),
                 None,'Served: 4'),
 '06_multi_hospital': (scenario([[1,1],[1,1]],['NP 1 1 1 30','SP 1 2 2 21','EP 2 3 1 10 5','EP 2 4 2 20 7','NP 3 5 2 10','SP 3 6 1 10'],['2 1 1','5 2 2'],rate=.15),
                       None,'Patients: 6'),
}

def run(input_path,output_path,seed=42,stdin=None):
    args=[str(EXE),str(input_path),str(output_path),str(seed)] if stdin is None else [str(EXE)]
    return subprocess.run(args,input=stdin,text=True,capture_output=True,timeout=20,cwd=ROOT)

def rows(text):
    return [tuple(map(int,line.split())) for line in text.splitlines()[1:] if line and line[0].isdigit()]

def main():
    SAMPLES.mkdir(exist_ok=True)
    with tempfile.TemporaryDirectory(dir=ROOT/'tests') as tmp:
        temp=Path(tmp)
        for name,(content,expected,stat) in cases.items():
            inp=SAMPLES/(name+'.txt'); inp.write_text(content)
            output=temp/(name+'.out.txt')
            result=run(inp,output)
            assert result.returncode==0,(name,result.stderr)
            report=output.read_text()
            actual=rows(report)
            if expected is not None: assert actual==expected,(name,actual,expected)
            assert stat in report,(name,report)
            assert len({r[1] for r in actual})==len(actual),'Patient finished twice'
            assert [r[0] for r in actual]==sorted(r[0] for r in actual),'Finish list out of order'
            assert all(ft>=qt and wt>=0 and ft>=qt+wt for ft,_,qt,wt in actual)
            if name=='05_failures':
                assert 'OUT failure events: 0\n' not in report
                interactive=run(inp,temp/'interactive.txt',stdin='1\n'+str(inp)+'\n'+str(temp/'interactive.txt')+'\n42\n'+'\n'*1000)
                assert interactive.returncode==0,interactive.stderr
                assert 'checkup)' in interactive.stdout,'Checkup list was never populated'
            # Golden outputs use seed 42; regenerate with the documented compiler/runtime.
            (SAMPLES/(name+'.out.txt')).write_text(report)
            print('PASS',name)
        invalid={
            'missing':'1\n',
            'zero_speed':scenario([[1,1]],[],speeds=(0,10)),
            'wrong_hospital':scenario([[1,1]],['NP 1 1 2 10']),
            'duplicate_id':scenario([[1,1]],['NP 1 1 1 10','NP 2 1 1 10']),
            'no_fleet':scenario([[0,0]],['NP 1 1 1 10']),
            'unsorted':scenario([[1,1]],['NP 2 1 1 10','NP 1 2 1 10']),
            'invalid_rate':scenario([[1,1]],[],rate=1.2),
        }
        for name,content in invalid.items():
            inp=temp/(name+'.txt'); inp.write_text(content)
            result=run(inp,temp/'invalid.out')
            assert result.returncode!=0 and 'Error:' in result.stderr,(name,result)
        empty=temp/'empty.txt'; empty.write_text(scenario([[0,0]],[]))
        assert run(empty,temp/'empty.out').returncode==0
        assert run(temp/'absent.txt',temp/'absent.out').returncode!=0
        # A normal request with no compatible car may still terminate by cancellation.
        cancelled=temp/'cancel_only.txt'
        cancelled.write_text(scenario([[0,0]],['NP 1 1 1 10'],['2 1 1']))
        assert run(cancelled,temp/'cancel_only.out').returncode==0
        assert 'Cancelled: 1' in (temp/'cancel_only.out').read_text()
        # Check exact recovery precedence and maintenance release at timestep 3.
        retry=temp/'retry.txt'
        retry.write_text(scenario([[0,1]],['EP 1 1 1 20 1','EP 2 2 1 10 99'],rate=.5))
        retryout=temp/'retry.out'
        interaction='1\n'+str(retry)+'\n'+str(retryout)+'\n42\n'+'\n'*1000
        recovery=run(retry,retryout,stdin=interaction)
        assert recovery.returncode==0,recovery.stderr
        at1=recovery.stdout.split('TIMESTEP 1 ')[1].split('TIMESTEP 2 ')[0]
        at3=recovery.stdout.split('TIMESTEP 3 ')[1].split('TIMESTEP 4 ')[0]
        assert 'Checkup (1): NC1_H1(checkup)' in at1
        assert 'OUT (1): NC1_H1_P1' in at3 and 'EP (1): 2' in at3
        backlog=temp/'backlog.txt'
        backlog.write_text(scenario([[0,1],[0,1],[0,1]],[
            'EP 1 1 1 100 100','EP 1 2 2 100 100','EP 1 3 3 100 100',
            'EP 1 4 1 10 90','EP 1 5 2 10 2','EP 1 6 2 10 1'],matrix=[[0,1,20],[1,0,20],[20,20,0]]))
        backlogout=temp/'backlog.out'
        trace=run(backlog,backlogout,stdin='1\n'+str(backlog)+'\n'+str(backlogout)+'\n42\nr\n\n')
        assert trace.returncode==0,trace.stderr
        hospital3=trace.stdout.split('Hospital 3\n')[1].split('OUT (')[0]
        assert 'EP (1): 4' in hospital3,hospital3
        before=empty.read_text(); assert run(empty,empty).returncode!=0; assert empty.read_text()==before
        # EOF exits cleanly; invalid menu input does not fall into a mode accidentally.
        menu=subprocess.run([str(EXE)],input='x\n0\n',text=True,capture_output=True,cwd=ROOT,timeout=5)
        assert menu.returncode==0 and 'Please enter 0, 1, or 2.' in menu.stdout
        print('PASS recovery order, checkup release, transfer backlogs, cancellation-only fleet')
        print('PASS validation, empty workload, file protection, and menu checks')

if __name__=='__main__': main()
